#pragma once

#include <random>
#include <concepts>
#include <functional>
#include <limits>

#include <Eigen/Eigen>

namespace controller {

/**
 * @brief A vector of booleans.
 */
using VectorXb = Eigen::Array<bool, Dynamic, 1> ArrayXb;

/**
 * @brief The clock used to keep track of time.
 */
using Clock = std::chrono::high_resolution_clock;

/**
 * @brief The time in seconds.
 */
using Time = std::chrono::seconds;

/**
 * @brief A duration in seconds.
 */
using Duration = std::chrono::seconds;

/**
 * @brief Get the current time.
 * @returns The timestamp at the current time.
 */
// inline Time now() {
//     return Clock::now().time_since_epoch();
// }

/**
 * @brief Configuration parameters for the controller.
 */
struct Configuration
{
    /// The number of rollouts to perform on each time.
    std::size_t rollouts;

    /// The time increment passed to the dynamics simulation when performing
    /// rollouts in seconds.
    double step_size;

    /// The duration of time of each rollout in seconds.
    double horison;
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
    virtual void step(const Control &control, double dt) = 0;

    /**
     * @brief Get the state of the dynamics simulation.
     * @returns The current state of the simulation.
     */
    virtual const State &state() = 0;
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
    std::unique_ptr<Trajectory<DynamicsType, CostType>> create(
        std::shared_ptr<DynamicsType> &dynamics,
        std::shared_ptr<CostType> &cost,
        const State &state,
        const Configuration &configuration
    );

    const Eigen::MatrixXd &optimal(
        const State &state,
        Time time
    );

private:

    void sample();

    /**
     * @brief Rollout from the current state.
     * 
     * @param index The rollout index from zero to 
     * @returns If the rollout converged.
     */
    void rollout(int index);

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
    Eigen::ArrayXd m_timestamps;

    /// The control parameters applied at each step in the rollouts.
    Eigen::MatrixXd m_controls;

    /// The evolution of states for each rollout.
    Eigen::MatrixXd m_states;

    /// The cost of each rollout.
    Eigen::MatrixXd m_costs;

    /// The weight of each rollout. The higher the better.
    Eigen::VectorXd m_weights;

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

    return std::make_unique<Trajectory<DynamicsType, CostType>>(
        dynamics,
        cost,
        state,
        configuration
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
}

template<typename DynamicsType, typename CostType>
const Eigen::MatrixXd &Trajectory<DynamicsType, CostType>::optimal(
    const State &state,
    Time time
) {
    m_current_state = state;

    // Calculate the timestamps at each time increment.
    for (int i = 0; i < m_steps; ++i)
        m_timestamps[i] = time + i * m_configuration.step_size;

    // Sample trajectories. 
    sample();

    // Rollout
    for (int i = 0; i < m_configuration.rollouts; i++)
        rollout(i);

    optimise();
}

template<typename DynamicsType, typename CostType>
void Trajectory<DynamicsType, CostType>::sample()
{
    // Set all trajectories to random noise.
    m_controls.unaryExpr(
        [&](float){ return m_distribution(m_generator); }
    );

    // Add the previous optimal trajectory as the mean.
    m_controls += m_optimal_control.replicate(m_configuration.rollouts, 1);

    // TODO: Sample zero and negative previous optimal trajectory.
}

template<typename DynamicsType, typename CostType>
bool Trajectory<DynamicsType, CostType>::rollout(int i)
{
    const DynamicsType &dynamics = *m_dynamics;
    const CostType &cost = *m_cost;

    dynamics.set(m_current_state);
    cost.reset();

    for (int step = 0; step < m_steps; ++step) {

        // Get a reference to the control parameters at this time step.
        auto control = m_controls.block<ControlDoF, 1>(i * ControlDoF, step);

        // Step the dynamics simulation and store.
        dynamics.step(m_configuration.step_size);
        m_states.block<StateDoF, 1>(i * StateDoF, step) = dynamics.state();

        double cost = m_cost.step(dynamics.state(), control, m_timestamps[step]);
        double discounted = cost * std::pow(m_configuration.discount_factor, step);

        if (std::isnan(discounted)) {
            m_costs[m_steps] = discounted;
            return;
        }

        // Cumulative running cost.
        m_costs[i] = cost + m_costs[i - 1];
    }
}

template<typename DynamicsType, typename CostType>
void Trajectory<DynamicsType, CostType>::optimise()
{
    double maximum = std::numeric_limits<double>::max();
    double minimum = std::numeric_limits<double>::min();

    // The cumulative costs at the final column.
    auto costs = m_costs.col(m_steps);
    auto [minimum, maximum] = std::minmax_element(
        costs.begin(),
        costs.end(),
        [](double a, double b) { return a < b : std::isnan(a); }
    );

    for (std::size_t rollout = 0; rollout < m_configuration.rollouts; ++rollout) {
        double cost = costs[rollout];

        // NaNs indicate a failed rollout and do not contribute to weight. 
        if (std::isnan(cost)) {
            m_weights[rollout] = 0.0;
            continue;
        }
        
    }


    m_weights.
}

} // namespace controller
