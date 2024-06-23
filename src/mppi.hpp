#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <random>
#include <iostream>

#include <Eigen/Eigen>

namespace mppi {

/**
 * @brief Configuration parameters for the controller.
 */
struct Configuration
{
    /// The number of rollouts to perform on each time.
    std::int64_t rollouts;

    /// The number of best control trajectory rollouts to keep to warmstart the
    // next control trajectory sampling phase (before rolling out).
    std::int64_t rollouts_cached;

    /// The time increment passed to the dynamics simulation when performing
    /// rollouts in seconds.
    double step_size;

    /// The duration of time of each rollout in seconds.
    double horison;

    /// The factor by which the optimal policy is updated.
    double gradient_step;

    /// The gradient is clipped to [-gradient_minmax, gradient_minmax].
    double gradient_minmax;

    /// Cost to likelihood mapping scaling.
    double cost_scale;

    /// Discount factor of cost calculation.
    double cost_discount_factor;

    /// Variance matrix of 
    Eigen::MatrixXd sample_variance;

    /// True to use the last control when a trajectory finishes, or false to use
    // the default value.
    bool control_default_last;

    /// If not control_default_last, the control to return when the trajectory
    // finishes.
    Eigen::VectorXd control_default_value;
};

/**
 * @brief The dynamics class stores and updates the system state.
 * 
 * This class should be subclassed to implement dynamics.
 */
class Dynamics
{
public:

    virtual ~Dynamics() = default;

    /**
     * @brief Get the degrees of freedom of the system control input.
     * @returns The degrees of freedom of the system control input.
     */
    virtual constexpr int control_dof() = 0;

    /**
     * @brief Get the degrees of freedom of the system state.
     * @returns The degrees of freedom of the system state.
     */
    virtual constexpr int state_dof() = 0;

    /**
     * @brief Set the dynamics simulation to a given state.
     * 
     * @param state The system state.
     * @param control The controls applied to the current state.
     */
    virtual void set(const Eigen::VectorXd &state) = 0;

    /**
     * @brief Step the dynamics simulation.
     * 
     * This function updates the internal state returned by Dynamics::state().
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    virtual Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) = 0;
};

/**
 * @brief The cost class stores and updates the cost of a rollout.
 * 
 * This class should be subclassed to implement the objective function.
 */
class Cost
{
public:

    virtual ~Cost() = default;

    /**
     * @brief Get the expected degrees of freedom of the system control input.
     * @returns The expected degrees of freedom of the system control input.
     */
    virtual constexpr int control_dof() = 0;

    /**
     * @brief Get the expected degrees of freedom of the system state.
     * @returns The expected degrees of freedom of the system state.
     */
    virtual constexpr int state_dof() = 0;

    /**
     * @brief Get the cost of a state and control input over dt.
     * 
     * @param state The state of the system.
     * @param control The control parameters applied to the state.
     * @param dt The change in time.
     * 
     * @returns The cost of the step.
     */
    virtual double get(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        double dt
    ) = 0;
};

/**
 * @brief A model predictive path integral (MPPI) optimal trajectory
 * generator.
 * 
 * Generates sequences of control outputs over a period of time (a control
 * trajectory) by optimising for the least cost randomly generated system state
 * evolution and control trajectory.
 * 
 * The generated trajectory should be linearly interpolated with respect to
 * time.
 * 
 * Data is stored in a column major fashion since that is the default in Eigen.
 * That is each number in a column is stored contiguously. Hence each control
 * vector and state vector are stored column wise. Time increases with the
 * column index.
 */
class Trajectory
{
public:

    /**
     * @brief Create a new Trajectory optimal trajectory generator.
     * 
     * The number of steps taken for each rollout is given by
     * round_down(rollout_duration / rollout_time_delta).
     * 
     * The total number of dynamics simulation increments is given by
     * rollouts * round_down(rollout_duration / rollout_time_delta).
     * 
     * @param dynamics A pointer to the dynamics object responsible that keeps
     * track of the current system state, and predicts future system states.
     * @param cost A pointer to a cost object, that tracks to cumulative cost
     * of dynamics simulation rollouts.
     * @param state The initial system state, to be passed to the dynamics
     * object.
     * @param rollout_time_delta The time increment passed to the dynamics
     * simulation when performing rollouts, in seconds.
     * @param time_duration The duration of time of each rollout, in seconds.
     * @param rollouts The number of rollouts to perform on each time.
     * 
     * @returns A pointer to the Trajectory trajectory generator on success, or
     * nullptr on failure.
     */
    static std::unique_ptr<Trajectory> create(
        Dynamics *dynamics,
        Cost *cost,
        const Configuration &configuration,
        const Eigen::VectorXd &state
    );

    /**
     * @brief Update the trajectory from a state and a time.
     * 
     * @param state The current state of the dynamics.
     * @param time The current time in seconds.
     */
    void update(const Eigen::VectorXd &state, double time);

    /**
     * @brief Get the times of each incremental update in the trajectory.
     * @returns The times of each trajectory step.
     */
    inline const Eigen::VectorXd &times() const {
        return m_times;
    }

    /**
     * @brief Get the control trajectory of a rollout.
     * 
     * @param rollout The rollout to get the control trajectory of.
     * @return The control trajectory of the rollout.
     */
    inline auto controls(std::int64_t rollout) const {
        return m_controls.block(
            rollout * m_dynamics->control_dof(), // start row
            0, // start col
            m_dynamics->control_dof(), // rows
            m_steps // cols
        );
    };

    /**
     * @brief Get the system state evolution of a rollout.
     * 
     * Each state corresponds to the time returned by time().
     * 
     * @param rollout The rollout to get the states of.
     * @returns The state evolution of the rollout.
     */
    // inline auto states(std::int64_t rollout) const {
    //     return m_states.block(
    //         m_configuration.rollouts,
    //         m_dynamics->control_dof(),
    //         0,
    //         rollout * m_dynamics->control_dof()
    //     );
    // };

    /**
     * @brief Get the costs of each rollout.
     * 
     * @param rollout The rollout to get the cost of.
     * @returns The cost of the rollout.
     */
    inline double cost(std::int64_t rollout) const {
        return m_costs[rollout];
    }

    /**
     * @brief Get the current optimal control trajectory.
     * 
     * Each control corresponds to the time returned by times().
     * 
     * @returns The current optimal control trajectory.
     */
    inline const Eigen::VectorXd &controls() const {
        return m_optimal_control;
    }

    /**
     * @brief Evaluate the current optimal control trajectory at a given time.
     * 
     * @param t The time to evaluate the control trajectory.
     * @returns The control parameters for the current time.
     */
    Eigen::VectorXd get(double t) const;

    /**
     * @brief Evaluate the current optimal control trajectory at a given time.
     * 
     * @param t The time to evaluate the control trajectory.
     * @returns The control parameters for the current time.
     */
    inline Eigen::VectorXd operator()(double t) const {
        return get(t);
    }

private:

    /**
     * @brief Initialise the trajectory generator.
     * 
     * See Trajectory::create().
     */
    Trajectory(
        Dynamics *dynamics,
        Cost *cost,
        const Configuration &configuration,
        const Eigen::VectorXd &state,
        int steps
    ) noexcept;

    /**
     * @brief Sample the control trajectories to simulate.
     * 
     * This step involves sampling a multivariate gaussian distribution multiple
     * times to fill out multiple different control trajectories stored in
     * m_controls. These are then simulated in rollout() and analysed in
     * optimise() to get the optimal trajectory.
     */
    void sample();

    /**
     * @brief Rollout a sampled control parameter trajectory.
     * 
     * The control trajectory for the given rollout is simulated with the system
     * dynamics and a final cost of those control actions is calculated.
     * 
     * @param index The rollout to perform.
     */
    void rollout(std::int64_t index);

    /**
     * @brief Update the optimal control trajectory.
     * 
     * Weighs each rollout according to its cost and gradient steps the previous
     * optimal control trajectory towards the next one.
     */
    void optimise();

    /// Keeps track of system state and simulates responses to control actions.
    Dynamics *m_dynamics;

    /// Keeps track of individual cumulative cost of dynamics simulation rollouts.
    Cost *m_cost;

    /// The configuration of the trajectory generation.
    Configuration m_configuration;

    /// The random number generator to use in the normal distribution.
    std::mt19937 m_generator;

    /// Distribution to use for noise.
    std::normal_distribution<double> m_distribution;

    /// The number of time steps per rollout.
    int m_steps;

    /// The number of degrees of freedom for the system state.
    int m_state_dof;

    /// The number of degrees of freedom for the control input.
    int m_control_dof;

    /// The current state from which the controller is generating trajectories.
    Eigen::VectorXd m_current_state;

    /// Timestamps of each rollout step.
    Eigen::VectorXd m_times;

    /// The control parameters applied at each step in the rollouts.
    Eigen::MatrixXd m_controls;

    /// The evolution of states for each rollout.
    // Eigen::MatrixXd m_states;

    /// The cost of each rollout.
    Eigen::VectorXd m_costs;

    /// The weight of each rollout. The higher the better.
    Eigen::VectorXd m_weights;

    /// The gradient applied to the optimal control trajectory.
    Eigen::MatrixXd m_gradient;

    /// The optimal control trajectory.
    Eigen::MatrixXd m_optimal_control;
};

} // namespace mppi
