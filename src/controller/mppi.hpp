#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <mutex>
#include <iostream>
#include <optional>
#include <chrono>

#include <Eigen/Eigen>

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

    virtual ~Dynamics() = default;

    /**
     * @brief Get the degrees of freedom for the dynamics control input.
     * @returns The degrees of freedom of the dynamics control input.
     */
    virtual constexpr int control_dof() = 0;

    /**
     * @brief Get the degrees of freedom of the dynamics state.
     * @returns The degrees of freedom of the dynamics state.
     */
    virtual constexpr int state_dof() = 0;

    /**
     * @brief Set the dynamics to a given parameterised state.
     * @param state The system state.
     */
    virtual void set(const Eigen::VectorXd &state) = 0;

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
     * @brief Make a copy of this parameterised dynamics.
     * 
     * This function is used to make copies of the parameterised dynamics for
     * each thread performing rollouts.
     * 
     * @returns A std::unique_ptr to the parameterised dynamics copy.
     */
    virtual std::unique_ptr<Dynamics> copy() = 0;
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

    virtual ~Cost() = default;

    /**
     * @brief Get the expected degrees of freedom of the dynamics control input.
     * @returns The expected degrees of freedom of the dynamics control input.
     */
    virtual constexpr int control_dof() = 0;

    /**
     * @brief Get the expected degrees of freedom of the dynamics state.
     * @returns The expected degrees of freedom of the dynamics state.
     */
    virtual constexpr int state_dof() = 0;

    /**
     * @brief Reset the cost.
     */
    virtual void reset() = 0;

    /**
     * @brief Get the cost of a dynamics state with control input over dt.
     * 
     * @param state The state of the dynamics system.
     * @param control The control parameters applied to the dynamics state.
     * @param time The current time.
     * 
     * @returns The cost of the step.
     */
    virtual double get(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        double time
    ) = 0;

    /**
     * @brief Make a copy of this objective function.
     * 
     * This function is used to make copies of the objective function for each
     * thread performing rollouts.
     * 
     * @returns A std::unique_ptr to the objective function copy.
     */
    virtual std::unique_ptr<Cost> copy() = 0;
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
     * @brief Configuration parameters for the controller.
     */
    struct Configuration
    {
        /// The dynamics object that keeps track of the current system state,
        /// and predicts future system states.
        std::unique_ptr<Dynamics> dynamics;

        /// A pointer to a cost obdject, that tracks to cumulative cost of
        /// dynamics simulation rollouts.
        std::unique_ptr<Cost> cost;

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
        struct Filter {

            /// The number of samples in the filter window.
            unsigned int window;

            /// The order of the polynomial used to fit data.
            unsigned int order;
        };

        /// If the trajectory should be filtered.
        std::optional<Filter> filter;

        /// The number of threads to use for concurrent work such as sampling and
        /// rollouts.
        unsigned int threads;
    };

    /**
     * @brief A rollout stores the noise applied to the current optimal
     * trajectory, and a measure of how good the noise is.
     */
    struct Rollout
    {
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

        /// The noise applied to the optimal rollout. Has `control_dof` number of
        /// rows and `steps` number of columns.
        Eigen::MatrixXd noise;

        /// The cost of the optimal rollout.
        double cost;
    };

    /// The number of always available precomputed rollouts. These are the zero
    /// control sample and negative of the previous optimal trajectory.
    static const constexpr std::int64_t s_static_rollouts = 2;

    /**
     * @brief Create a new Trajectory optimal trajectory generator.
     * 
     * @param configuration The configuration of the trajectory.
     * 
     * @returns A pointer to the trajectory on success, or nullptr on failure.
     */
    static std::unique_ptr<Trajectory> create(const Configuration &configuration);

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

    inline double get_time_step() const {
        return m_time_step;
    }

    inline std::size_t get_step_count() const {
        return m_step_count;
    }

    inline double get_update_duration() const {
        return m_update_duration;
    }

    inline double get_update_last() const {
        return m_update_last;
    }

    inline std::size_t get_update_count() const {
        return m_update_count;
    }

    inline const auto &get_weights() const {
        return m_weights;
    }

    inline const auto &get_gradient() const {
        return m_gradient;
    }

    inline const auto &get_rollouts() const {
        return m_rollouts;
    }

    inline const auto &get_optimal_rollout() const {
        return m_optimal_control;
    }

    // inline const auto &get_update_delta() {
    //     return m_update_delta;
    // }

    /**
     * 
     * @brief Get the noise trajectory of a rollout.
     * 
     * A block of data where each column is a vector with control dof elements.
     * Each vector contains noise applied to the matching optimal control
     * input for the given time step. The number of columns is equal to the
     * number of time steps.
     * 
     * @param rollout The rollout to get the noise of.
     * @return The noise of the rollout.
     */
    inline auto get_rollout(std::int64_t rollout) {
        return m_rollouts[rollout];
    };

    /**
     * @brief Get the optimal trajectory starting at the last update time.
     * @returns The optimal control trajectory.
     */
    inline const Eigen::VectorXd &trajectory() const {
        return m_optimal_control;
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
    Trajectory(const Configuration &configuration) noexcept;

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
    void rollout(Rollout &rollout, Dynamics &dynamics, Cost &cost);

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

    /// The previous optimal control, shifted to align with the current time.
    Eigen::MatrixXd m_optimal_control_shifted;

    /// The optimal control.
    Eigen::MatrixXd m_optimal_control;

    /// Mutex protecting concurrent access to the optimal control.
    std::mutex m_optimal_control_mutex;

    /// The number of best rollouts to keep for warm starting the next iteration.
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
