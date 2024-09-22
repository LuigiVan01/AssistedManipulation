#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <mutex>
#include <iostream>
#include <optional>
#include <chrono>

#include <Eigen/Eigen>

#include "controller/json.hpp"
#include "controller/gaussian.hpp"
#include "controller/concurrency.hpp"
#include "controller/filter.hpp"

namespace mppi {

using namespace std::chrono_literals;

/**
 * @brief Parameterised system dynamics.
 * 
 * Parameterised meaning the whole dynamics state can be encoded by a finite
 * vector of parameters. For example, a simple car can be parameterised by
 * (x, y, yaw, vx, vy, omega).
 * 
 * Subclass to implement dynamics.
 */
class Dynamics
{
public:

    /**
     * @brief Virtual destructor for derived classes.
     */
    virtual ~Dynamics() = default;

    /**
     * @brief Make a copy of this parameterised dynamics.
     * 
     * This function is used to make copies of the parameterised dynamics for
     * each thread performing rollouts.
     * 
     * @returns A std::unique_ptr to the parameterised dynamics copy.
     */
    virtual std::unique_ptr<Dynamics> copy() = 0;

    /**
     * @brief Update the parameters of the dynamics state with a step in time,
     * with a control input applied.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     * 
     * @returns The parameterised state of the system.
     */
    virtual Eigen::Ref<Eigen::VectorXd> step(
        const Eigen::VectorXd &control,
        double dt
    ) = 0;

    /**
     * @brief Set the dynamics to a given parameterised state.
     * 
     * @param state The system state.
     * @param time The time of the state.
     */
    virtual void set_state(const Eigen::VectorXd &state, double time) = 0;

    /**
     * @brief Get the dynamics state.
     * @returns The state of the dynamics.
     */
    virtual Eigen::Ref<Eigen::VectorXd> get_state() = 0;

    /**
     * @brief Get the degrees of freedom for the dynamics control input.
     * @returns The degrees of freedom of the dynamics control input.
     */
    virtual constexpr int get_control_dof() = 0;

    /**
     * @brief Get the degrees of freedom of the dynamics state.
     * @returns The degrees of freedom of the dynamics state.
     */
    virtual constexpr int get_state_dof() = 0;
};

/**
 * @brief An objective function that rates how good a sequence of control inputs
 * is.
 * 
 * Subclass to implement behaviour.
 */
class Cost
{
public:

    /**
     * @brief Virtual destructor for derived classes.
     */
    virtual ~Cost() = default;

    /**
     * @brief Make a copy of this objective function.
     * 
     * This function is used to make copies of the objective function for each
     * thread performing rollouts.
     * 
     * @returns A std::unique_ptr to the objective function copy.
     */
    virtual std::unique_ptr<Cost> copy() = 0;

    /**
     * @brief Reset the cost.
     */
    virtual void reset() = 0;

    /**
     * @brief Get the cost of a dynamics state with control input over dt.
     * 
     * @param state The state of the dynamics system.
     * @param control The control parameters applied to the dynamics state.
     * @param dynamics Pointer to the dynamics at the time.
     * @param time The current time.
     * 
     * @returns The cost of the step.
     */
    virtual double get_cost(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        Dynamics *dynamics,
        double time
    ) = 0;

    /**
     * @brief Get the expected degrees of freedom of the dynamics control input.
     * @returns The expected degrees of freedom of the dynamics control input.
     */
    virtual constexpr int get_control_dof() = 0;

    /**
     * @brief Get the expected degrees of freedom of the dynamics state.
     * @returns The expected degrees of freedom of the dynamics state.
     */
    virtual constexpr int get_state_dof() = 0;
};

/**
 * @brief A filter applied to a trajectory per timestep.
 */
class Filter
{
public:

    /**
     * @brief Apply the filter.
     * 
     * @param state The state of the system.
     * @param control The controls of the system.
     * @param time The time.
     * 
     * @returns The filtered control. 
     */
    virtual Eigen::VectorXd filter(
        Eigen::Ref<Eigen::VectorXd> state,
        Eigen::Ref<Eigen::VectorXd> control,
        double time
    ) = 0;

    /**
     * @brief Reset the filter.
     * 
     * @param state The state of the system.
     * @param time The time.
     */
    virtual void reset(Eigen::Ref<Eigen::VectorXd> state, double time) = 0;
};

/**
 * @brief Configuration parameters of the mppi trajectory.
 */
struct Configuration
{
    /// The initial state of the system.
    Eigen::VectorXd initial_state;

    /// The number of rollouts to perform on each trajectory update.
    std::int64_t rollouts;

    /// The number of best rollouts to keep to warmstart the next rollout.
    std::int64_t keep_best_rollouts;

    /// The time step of the rollout dynamics simulations, in seconds.
    double time_step;

    /// The duration of time in each rollout, in seconds.
    double horison;

    /// The factor by which the optimal policy is updated with the gradient.
    double gradient_step;

    /// Cost to likelihood mapping scaling.
    double cost_scale;

    /// Discount factor of cost calculation.
    double cost_discount_factor;

    /// The covariance matrix to generate rollout noise from.
    Eigen::MatrixXd covariance;

    /// If the control output is bounded between control_min and control_max.
    bool control_bound;

    /// The minimum control output for each control degree of freedom.
    Eigen::VectorXd control_min;

    /// The maximum control output for each control degree of freedom.
    Eigen::VectorXd control_max;

    /// The control to return when getting the trajectory past the horison.
    std::optional<Eigen::VectorXd> control_default;

    /// Filter configuration.
    struct Smoothing {

        /// The number of samples in the filter window.
        unsigned int window;

        /// The order of the polynomial used to fit data.
        unsigned int order;

        // JSON conversion for Smoothing.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Smoothing, window, order)
    };

    /// If the trajectory should be filtered.
    std::optional<Smoothing> smoothing;

    /// The number of threads to use for concurrent work such as sampling and
    /// rollouts.
    unsigned int threads;

    // JSON conversion for mppi configuration.
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        Configuration,
        initial_state, rollouts, keep_best_rollouts, time_step, horison,
        gradient_step, cost_scale, cost_discount_factor, covariance,
        control_bound, control_min, control_max, control_default, smoothing, threads
    )
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
     * @brief A rollout stores the noise applied to the current optimal
     * trajectory, and a measure of how good the noise is.
     */
    class Rollout
    {
    public:

        /// The noise applied to the previous optimal rollout. Has `control_dof`
        /// number of rows and `steps` number of columns.
        Eigen::MatrixXd noise;

        /// The cost of the rollout.
        double cost;

    private:

        friend class Trajectory;

        /**
         * @brief Create a new rollout.
         * 
         * @param control_dof The number of rows of noise.
         * @param steps The number of columns of noise.
         */
        Rollout(std::size_t control_dof, std::size_t steps)
            : noise(control_dof, steps)
            , cost(0.0)
        {}
    };

    /// The number of rollouts added to the configured rollouts. These are the
    /// zero control sample and negative of the previous optimal trajectory.
    static const constexpr std::int64_t s_static_rollouts = 2;

    /**
     * @brief Create a new Trajectory optimal trajectory generator.
     * 
     * @param configuration The configuration of the trajectory.
     * @param dynamics The dynamics object that keeps track of the current system
     * state and predicts future system states.
     * @param cost A pointer to a cost object that evaluates the objective
     * function at each rollout time step.
     * @param filter An optional filter to apply to the optimal rollout after
     * each update.
     * 
     * @returns A pointer to the trajectory on success, or nullptr on failure.
     */
    static std::unique_ptr<Trajectory> create(
        const Configuration &configuration,
        std::unique_ptr<Dynamics> &&dynamics,
        std::unique_ptr<Cost> &&cost,
        std::unique_ptr<Filter> &&filter = nullptr
    );

    /**
     * @brief Increments the generated trajectory towards the optimal one, given
     * the current state and time.
     * 
     * It is critical that this function is called with overlapping time
     * horisons. The more updates a single time step receives, the better the
     * control action at that time step is.
     * 
     * @param state The current state of the dynamics system.
     * @param time The current time in seconds. Must be monotonic.
     */
    void update(const Eigen::Ref<Eigen::VectorXd> state, double time);

    /**
     * @brief Get the state degrees of freedom.
     */
    inline unsigned int get_state_dof() const {
        return m_state_dof;
    }

    /**
     * @brief Get the control degrees of freedom.
     */
    inline unsigned int get_control_dof() const {
        return m_control_dof;
    }

    /**
     * @brief Get the trajectory time step.
     */
    inline double get_time_step() const {
        return m_time_step;
    }

    /**
     * @brief Get the number of trajectory time steps.
     */
    inline unsigned int get_step_count() const {
        return m_step_count;
    }

    /**
     * @brief Get the computation duration of the last update.
     */
    inline double get_update_duration() const {
        return m_update_duration;
    }

    /**
     * @brief Get the time of the last update.
     */
    inline double get_update_last() const {
        return m_update_last;
    }

    /**
     * @brief Get the cumulative number of trajectory updates.
     */
    inline std::size_t get_update_count() const {
        return m_update_count;
    }

    /**
     * @brief Get the number of rollouts.
     */
    inline std::size_t get_rollout_count() const {
        return m_rollout_count;
    }

    /**
     * @brief Get the initial state of all the rollouts of the previous update
     * (or the initial state if update has not been called yet).
     */
    inline const auto &get_rolled_out_state() const {
        return m_rollout_state;
    }

    /**
     * @brief Get the last update rollout weights.
     */
    inline const Eigen::VectorXd &get_weights() const {
        return m_weights;
    }

    /**
     * @brief Get the last update gradient.
     */
    inline const Eigen::MatrixXd &get_gradient() const {
        return m_gradient;
    }

    /**
     * @brief Get the rollouts.
     */
    inline const std::vector<mppi::Trajectory::Rollout> &get_rollouts() const {
        return m_rollouts;
    }

    /**
     * @brief Get the optimal rollout 
     */
    inline const Eigen::MatrixXd &get_optimal_rollout() const {
        return m_optimal_control;
    }

    /**
     * @brief Get the optimal trajectory starting at the last update time.
     * @returns The optimal control trajectory.
     */
    inline const Eigen::MatrixXd &trajectory() const {
        return m_optimal_control;
    }

    /**
     * @brief Get the optimal rollout cost.
     */
    inline double get_optimal_total_cost() const {
        return m_optimal_rollout.cost;
    }

    inline const Cost &get_optimal_cost() const {
        return *m_cost[0];
    }

    inline const Dynamics &get_optimal_dynamics() const {
        return *m_dynamics[0];
    }

    /**
     * @brief Evaluate the optimal control trajectory at a given time.
     * 
     * @param control Reference to the control vector to fill.
     * @param t The time to evaluate the control trajectory.
     */
    void get(Eigen::Ref<Eigen::VectorXd> control, double time);

    /**
     * @brief Evaluate the current optimal control trajectory at a given time.
     * 
     * @param time The time to evaluate the control trajectory.
     * @returns The control parameters for the current time.
     */
    inline Eigen::VectorXd operator()(double time) {
        Eigen::VectorXd control;
        get(control, time);
        return control;
    }

private:

    /**
     * @brief Initialise the trajectory generator.
     * @param configuration The configuration of the trajectory generator.
     */
    Trajectory(
        const Configuration &configuration,
        std::unique_ptr<Dynamics> &&dynamics,
        std::unique_ptr<Cost> &&cost,
        std::unique_ptr<Filter> &&filter
    ) noexcept;

    /**
     * @brief Sample the rollouts to simulate.
     * 
     * Samples noise from a multivariate gaussian distribution for each time
     * step of each rollout.
     * 
     * Shifts the previous rollouts forward in time to align with the current
     * sample time. Otherwise the optimal rollout would be applied after a time
     * delay.
     * 
     * The best k rollouts from the previous sample are not resampled. Only the
     * part of the horison that has come into view are sampled.
     * 
     * Each rollout noise is added to the current optimal rollout and simulated
     * in rollout(), and analysed in optimise() to get the optimal trajectory.
     */
    void sample(double time);

    /**
     * @brief Distributes rollout calculations amongst worker threads, and waits
     * for them to complete.
     */
    void rollout();

    /**
     * @brief Rollout from the current state.
     * 
     * @param rollout The rollout to store data in.
     * @param dynamics The dynamics object to use for rolling out.
     * @param cost The objective function to use to calculate rollout cost.
     */
    void rollout(Rollout *rollout, Dynamics *dynamics, Cost *cost);

    /**
     * @brief Updates the optimal control trajectory.
     * 
     * Transforms the costs of each simulated rollouts into normalised
     * likelihoods.
     * 
     * Creates a linear combination of all rollout noise, using the normalised
     * likelihoods as the weights, to produce the optimal rollout noise.
     * 
     * Optionally smooth filters the optimal noise.
     * 
     * Uses the optimal rollout noise to gradient step the optimal control
     * trajectory.
     */
    void optimise();

    /**
     * @brief Rolls out and filters the optimal trajectory.
     * 
     * Also computes the optimal trajectory cost during rollout.
     */
    void filter();

    /// The number of time steps per rollout.
    const int m_step_count;

    /// The duration of each dynamics simulation step.
    const double m_time_step;

    /// The number of rollouts.
    const int m_rollout_count;

    /// The number of threads.
    const int m_thread_count;

    /// The number of degrees of freedom for the system state.
    const int m_state_dof;

    /// The number of degrees of freedom for the control input.
    const int m_control_dof;

    /// Time of last trajectory update, in seconds.
    double m_update_last;

    /// The duration of the last update, in seconds.
    double m_update_duration;

    /// The number of trajectory updates.
    std::size_t m_update_count;

    /// A collection of threads used for sampling and rollouts.
    ThreadPool m_thread_pool;

    /// Keeps track of system state and simulates responses to control actions.
    std::vector<std::unique_ptr<Dynamics>> m_dynamics;

    /// Keeps track of individual cumulative cost of dynamics simulation rollouts.
    std::vector<std::unique_ptr<Cost>> m_cost;

    /// Barrier to wait for thread concurrent rollout calculation to complete.
    std::vector<std::future<void>> m_futures;

    /// The random number generator to use in the normal distribution.
    Gaussian m_gaussian;

    /// The current state from which the controller is generating trajectories.
    Eigen::VectorXd m_rollout_state;

    /// The current time of trajectory generation.
    double m_rollout_time;

    /// The time of the last trajectory generation.
    double m_last_rollout_time;

    /// The number of time steps to shift control by to align with current time.
    std::int64_t m_shift_by;

    /// The number of columns that was shifted to align with current time.
    std::int64_t m_shifted;

    /// The discount factor of near future control actions.
    const double m_cost_discount_factor;

    /// Scaling applied to the cost to likelyhood mapping.
    const double m_cost_scale;

    /// The control parameters applied at each step in the rollouts.
    std::vector<Rollout> m_rollouts;

    /// The weight of each rollout. The higher the better.
    Eigen::VectorXd m_weights;

    /// The gradient applied to the optimal control trajectory.
    Eigen::MatrixXd m_gradient;

    /// The scalar amount by which to add the gradient to the optimal control.
    const double m_gradient_step;

    /// The filter to apply to the optimal trajectory. May be nullptr.
    std::unique_ptr<Filter> m_filter;

    /// The previous optimal control, shifted to align with the current time.
    Eigen::MatrixXd m_optimal_control_shifted;

    /// The rollout of the optimal trajectory. The filter is applied here.
    Rollout m_optimal_rollout;

    /// The optimal control.
    Eigen::MatrixXd m_optimal_control;

    /// Mutex protecting concurrent access to the optimal control.
    std::mutex m_optimal_control_mutex;

    /// The number of best rollouts to keep for warm starting the next update.
    const std::int64_t m_keep_best_rollouts;

    /// Buffer to store rollout indexes before sorting them by cost.
    std::vector<std::int64_t> m_ordered_rollouts;

    /// The smoothing filter used on the optimal control noise, if enabled.
    std::optional<SavitzkyGolayFilter> m_smoothing_filter;

    /// If the trajectory should be bounded each time step.
    const bool m_bound_control;

    /// The minimum control if bounded.
    Eigen::VectorXd m_control_min;

    /// The maximum control if bounded.
    Eigen::VectorXd m_control_max;

    /// If the end of the trajectory is reached, the control to return.
    std::optional<Eigen::VectorXd> m_control_default;
};

} // namespace mppi
