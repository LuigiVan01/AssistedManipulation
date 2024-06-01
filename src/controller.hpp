#pragma once

#include <random>
#include <concepts>
#include <functional>
#include <limits>
#include <memory>

#include <Eigen/Eigen>

namespace controller {

/**
 * @brief Configuration parameters for the controller.
 */
struct Configuration
{
    /// The number of rollouts to perform on each time.
    std::size_t rollouts;

    /// The number of best control trajectory rollouts to keep to warmstart the
    // next control trajectory sampling phase (before rolling out).
    std::size_t rollouts_cached;

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
 * This class should be subclassed to implement dynamics. The class must be
 * copy constructable.
 * 
 * @tparam StateDoF The number of degrees of freedom of the system.
 * @tparam ControlDoF The number of degrees of system of the controls.
 */
template<std::size_t StateDoF, std::size_t ControlDoF>
class Dynamics
{
public:

    /// The number of degrees of freedom of the system.
    static constexpr const std::size_t StateDoF = StateDoF;

    /// The number of degrees of system of the controls.
    static constexpr const std::size_t ControlDoF = ControlDoF;

    /// A state of the dynamics.
    using State = Eigen::Matrix<double, StateDoF, 1>;

    /// A control of the dynamics.
    using Control = Eigen::Matrix<double, ControlDoF, 1>;

    virtual ~Dynamics() = default;

    /**
     * @brief Set the dynamics simulation to a given state.
     * 
     * @param state The system state.
     * @param control The controls applied to the current state.
     * @param t The time in the simulation.
     */
    virtual void set(const State &state, double t) = 0;

    /**
     * @brief Step the dynamics simulation.
     * 
     * This function updates the internal state returned by Dynamics::state().
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    virtual State step(const Control &control, double dt) = 0;
};

/**
 * @brief The cost class stores and updates the cost of a rollout.
 * 
 * @tparam StateDoF 
 * @tparam ControlDoF 
 */
template<std::size_t StateDoF, std::size_t ControlDoF>
class Cost
{
public:

    static constexpr const std::size_t StateDoF = StateDoF;
    static constexpr const std::size_t ControlDoF = ControlDoF;

    virtual ~Cost() = default;

    /**
     * @brief Update the cost given a subsequent state from a contorl input.
     * 
     * @param state The next state of the system.
     * @param control The controls used to achieve the state.
     * @param t The change in time.
     */
    void step(
        const Eigen::Matrix<double, StateDoF, 1> &state,
        const Eigen::Matrix<double, ControlDoF, 1> &control,
        double dt
    );

    /**
     * @brief Reset the cost.
     */
    virtual void reset() = 0;

    /**
     * @brief Get the current cumulative cost.
     * @returns The current cumulative cost.
     */
    virtual double cost() = 0;
};

/**
 * @brief A model predictive path integral (MPPI) optimal trajectory
 * generator.
 * 
 * Generates sequences of control outputs over a period of time (a control
 * trajectory) by optimising for the least cost randomly generated system state
 * evolution and control trajectory.
 * The generated trajectory should be linearly interpolated with respect to
 * time.
 * 
 * Data is stored in a column major fashion since that is the default in Eigen.
 * That is each number in a column is stored contiguously. Hence each control
 * vector and state vector are stored column wise. Time increases with the
 * column index.
 * 
 * @tparam DynamicsType The dynamics type.
 * @tparam CostType The cost type.
 */
template<class DynamicsType, class CostType>
class Trajectory
{
public:

    // Ensure that the system and cost functions are suitable.
    static_assert(DynamicsType::StateDoF == CostType::StateDoF);
    static_assert(DynamicsType::ControlDoF == CostType::ControlDoF);

    using State = DynamicsType::State;
    using Control = DynamicsType::Control;

    static constexpr const std::size_t StateDoF = DynamicsType::StateDoF;
    static constexpr const std::size_t ControlDoF = DynamicsType::ControlDoF;

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
    static std::unique_ptr<Trajectory<DynamicsType, CostType>> create(
        std::shared_ptr<DynamicsType> &dynamics,
        std::shared_ptr<CostType> &cost,
        const State &state,
        const Configuration &configuration
    );

    /**
     * @brief Update the trajectory from a state and a time.
     * 
     * @param state The current state of the dynamics.
     * @param time The current time in seconds.
     */
    void update(const State &state, double time);

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
    inline auto controls(std::size_t rollout) const {
        return m_controls.block(
            m_configuration.rollouts,
            ControlDoF,
            0,
            rollout * ControlDoF
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
    inline auto states(std::size_t rollout) const {
        return m_states.block(
            m_configuration.rollouts,
            ControlDoF,
            0,
            rollout * ControlDoF
        );
    };

    /**
     * @brief Get the costs of each rollout.
     * 
     * @param rollout The rollout to get the cost of.
     * @returns The cost of the rollout.
     */
    inline double cost(std::size_t rollout) const {
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
    Control operator()(double t) const;

private:

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
    void rollout(std::size_t index);

    /**
     * @brief Update the optimal control trajectory.
     * 
     * Weighs each rollout according to its cost and gradient steps the previous
     * optimal control trajectory towards the next one.
     */
    void optimise();

    /**
     * @brief Initialise the trajectory generator.
     * 
     * See Trajectory::create().
     */
    Trajectory(
        std::shared_ptr<DynamicsType> &dynamics,
        std::shared_ptr<CostType> &cost,
        const Configuration &configuration,
        const State &state,
        int steps
    ) noexcept;

    /// Keeps track of system state and simulates responses to control actions.
    std::shared_ptr<Dynamics<StateDoF, ControlDoF>> m_dynamics;

    /// Keeps track of individual cumulative cost of dynamics simulation rollouts.
    std::shared_ptr<Cost<StateDoF, ControlDoF>> m_cost;

    /// The configuration of the trajectory generation.
    Configuration m_configuration;

    /// The random number generator to use in the normal distribution.
    std::mt19937 m_generator;

    /// Distribution to use for noise.
    std::normal_distribution<double> m_distribution;

    int m_steps;

    /// The current state from which the controller is generating trajectories.
    State m_current_state;

    /// Timestamps of each rollout step.
    Eigen::VectorXd m_times;

    /// The control parameters applied at each step in the rollouts.
    Eigen::MatrixXd m_controls;

    /// The evolution of states for each rollout.
    Eigen::MatrixXd m_states;

    /// The cost of each rollout.
    Eigen::VectorXd m_costs;

    /// The weight of each rollout. The higher the better.
    Eigen::VectorXd m_weights;

    /// The gradient applied to the optimal control trajectory.
    Eigen::MatrixXd m_gradient;

    /// The optimal control trajectory.
    Eigen::MatrixXd m_optimal_control;
};

template<typename DynamicsType, typename CostType>
std::unique_ptr<Trajectory<DynamicsType, CostType>>
Trajectory<DynamicsType, CostType>::create(
    std::shared_ptr<DynamicsType> &dynamics,
    std::shared_ptr<CostType> &cost,
    const State &state,
    const Configuration &configuration
) {
    // TODO: Validation goes here.
    if (configuration.rollouts < 1)
        return nullptr;

    int steps = std::ceil(configuration.horison / configuration.step_size);

    return std::unique_ptr<Trajectory<DynamicsType, CostType>>(
        new Trajectory<DynamicsType, CostType>(
            dynamics,
            cost,
            configuration,
            state,
            steps
        )
    );
}

template<typename DynamicsType, typename CostType>
Trajectory<DynamicsType, CostType>::Trajectory(
    std::shared_ptr<DynamicsType> &dynamics,
    std::shared_ptr<CostType> &cost,
    const Configuration &configuration,
    const State &state,
    int steps
) noexcept
  : m_dynamics(dynamics)
  , m_cost(cost)
  , m_configuration(configuration)
  , m_generator(std::random_device{}())
  , m_steps(steps)
  , m_current_state(state)
  , m_controls(steps, configuration.rollouts * ControlDoF)
  , m_states(steps, configuration.rollouts * StateDoF)
  , m_costs(1, configuration.rollouts)
  , m_optimal_control(steps, ControlDoF)
{
    m_gradient.setZero();
}

template<typename DynamicsType, typename CostType>
void Trajectory<DynamicsType, CostType>::update(const State &state, double time)
{
    m_current_state = state;

    // Calculate the timestamps at each time increment.
    for (int i = 0; i < m_steps; ++i)
        m_times[i] = time + i * m_configuration.step_size;

    // Sample control trajectories. 
    sample();

    // Rollout
    for (std::size_t i = 0; i < m_configuration.rollouts; i++)
        rollout(i);

    optimise();
}

template<typename DynamicsType, typename CostType>
void Trajectory<DynamicsType, CostType>::sample()
{
    // TODO: Warmstart sampling, zero velocity sample, negative previous
    // optimal trajectory.

    // Set all trajectories to random noise.
    m_controls.unaryExpr(
        [&](float){ return m_distribution(m_generator); }
    );

    // Relative to the previous optimal trajectory.
    m_controls += m_optimal_control.replicate(m_configuration.rollouts, 1);
}

template<typename DynamicsType, typename CostType>
void Trajectory<DynamicsType, CostType>::rollout(std::size_t i)
{
    m_dynamics.set(m_current_state);
    m_cost.reset();

    for (int step = 0; step < m_steps; ++step) {

        // Get a reference to the control parameters at this time step.
        auto control = m_controls.block<ControlDoF, 1>(i * ControlDoF, step);

        // Step the dynamics simulation and store.
        State state = m_dynamics->step(m_configuration.step_size);
        m_states.block<StateDoF, 1>(i * StateDoF, step) = m_dynamics->state();

        double cost = (
            m_cost.step(state, control, m_times[step]) *
            std::pow(m_configuration.cost_discount_factor, step)
        );

        if (std::isnan(cost)) {
            m_costs[m_steps] = NAN;
            return;
        }

        // Cumulative running cost.
        m_costs[i] += cost;
    }
}

template<typename DynamicsType, typename CostType>
void Trajectory<DynamicsType, CostType>::optimise()
{
    double maximum = std::numeric_limits<double>::max();
    double minimum = std::numeric_limits<double>::min();

    // Get the minimum and maximum rollout cost.
    auto [minimum, maximum] = std::minmax_element(
        m_costs.begin(),
        m_costs.end(),
        [](double a, double b) { return a < b ? true : std::isnan(b); }
    );

    // For parameterisation of each cost.
    double difference = maximum - minimum;

    // Running sum of total likelihood for normalisation between zero and one.
    double total = 0.0;

    // Transform the weights to likelihoods.
    for (std::size_t rollout = 0; rollout < m_configuration.rollouts; ++rollout) {
        double cost = m_costs[rollout];

        // NaNs indicate a failed rollout and do not contribute anything. 
        if (std::isnan(cost)) {
            m_weights[rollout] = 0.0;
            continue;
        }

        double likelihood = std::exp(
            m_configuration.cost_scale * (cost - minimum) / difference
        );

        total += likelihood;
        m_weights[rollout] = likelihood;
    }

    // Normalise the likelihoods.
    std::transform(
        m_weights.begin(),
        m_weights.end(),
        m_weights.begin(),
        [total](double likelihood){ return likelihood / total; }
    );

    // The optimal trajectory is the weighted samples.
    m_gradient.setZero();
    for (std::size_t rollout = 0; rollout < m_configuration.rollouts; ++rollout)
        m_gradient += controls(rollout) * m_weights[rollout];

    // Clip gradient.
    m_gradient = m_gradient
        .cwiseMax(m_configuration.gradient_minmax)
        .cwiseMin(m_configuration.gradient_minmax);

    m_optimal_control += m_gradient * m_configuration.gradient_step;
}

template<typename DynamicsType, typename CostType>
Trajectory<DynamicsType, CostType>::Control
Trajectory<DynamicsType, CostType>::operator()(double time) const
{
    auto it = std::upper_bound(m_times.begin(), m_times.end(), time);
    std::size_t index = std::distance(m_times.begin(), it);

    // Past the end of the trajectory. Return the specified default control
    // parameters.
    if (index = m_times.end()) {
        if (m_configuration.control_default_last)
            return m_optimal_control.lastCol();
        return m_configuration.control_default_value;
    }

    assert(m_times.length() > 1);
    auto previous = it - 1;

    // Parameterisation of the time between nearest two timestamps.
    double t = (time - *previous) / (*it - *previous);

    // Linear interpolation between optimal controls.
    return (
        (1.0 - t) * m_optimal_control.col(index - 1) +
        t * m_optimal_control.col(index)
    );
}

} // namespace controller
