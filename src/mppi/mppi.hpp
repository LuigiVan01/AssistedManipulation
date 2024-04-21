#pragma once

#include <concepts>
#include <functional>

#include <Eigen/Eigen>

namespace mpxx {

enum Status {
    OKAY,
    CONVERGENCE
};

/**
 * @brief The dynamics class stores and updates the system state.
 * 
 * This class should be subclassed to implement dynamics. The class must be
 * copy constructable.
 * 
 * @tparam SystemDoF The number of degrees of freedom of the system.
 * @tparam ControlDoF The number of degrees of system of the controls.
 */
template<std::size_t SystemDoF, std::size_t ControlDoF>
class Dynamics
{
public:

    /// The number of degrees of freedom of the system.
    static constexpr const std::size_t SystemDoF = SystemDoF;

    /// The number of degrees of system of the controls.
    static constexpr const std::size_t ControlDoF = ControlDoF;

    /// A state of the dynamics.
    using State = Eigen::Matrix<double, SystemDoF, 1>;

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
 * @tparam SystemDoF 
 * @tparam ControlDoF 
 */
template<std::size_t SystemDoF, std::size_t ControlDoF>
class Cost
{
public:

    static constexpr const std::size_t SystemDoF = SystemDoF;
    static constexpr const std::size_t ControlDoF = ControlDoF;

    /**
     * @brief Update the cost given a subsequent state from a contorl input.
     * 
     * @param state The next state of the system.
     * @param control The controls used to achieve the state.
     * @param t The change in time.
     */
    void step(
        const Eigen::Matrix<double, SystemDoF, 1> &state,
        const Eigen::Matrix<double, ControlDoF, 1> &control,
        double dt
    );

    /**
     * @brief Reset the cost.
     */
    void reset();

    /**
     * @brief Get the current cumulative cost.
     * @returns The current cumulative cost.
     */
    double cost();
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
 * @tparam SystemDoF The number of state parameters x.
 * @tparam ControlDoF The number of control parameters u.
 */
template<class DynamicsType, class CostType>
class MPPI
{
public:

    // Ensure that the system and cost functions are suitable.
    static_assert(DynamicsType::SystemDoF == CostType::SystemDoF);
    static_assert(DynamicsType::ControlDoF == CostType::ControlDoF);

    using State = DynamicsType::State;
    using Control = DynamicsType::Control;

    using StateDoF = DynamicsType::SystemDoF;
    using ControlDoF = DynamicsType::ControlDoF;

    /**
     * @brief Create a new MPPI optimal trajectory generator.
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
     * @returns A pointer to the MPPI trajectory generator on success, or
     * nullptr on failure.
     */
    std::unique_ptr<MPPI<DynamicsType, CostType>> create(
        const std::shared_ptr<DynamicsType> &dynamics,
        const std::shared_ptr<CostType> &cost,
        const State &state,
        double rollout_time_delta,
        double rollout_duration,
        std::size_t rollouts
    );

    const Eigen::MatrixXd &get_optimal_control_trajectory(const State &state);

private:

    /**
     * @brief Initialise the trajectory.
     * 
     * See MPPI::create().
     */
    MPPI(
        std::shared_ptr<DynamicsType> &&dynamics,
        std::shared_ptr<CostType> &&cost,
        const State &state,
        double rollout_time_delta,
        double rollout_duration,
        std::size_t rollouts
    ) noexcept;

    /// Keeps track of system state and simulates responses to control actions.
    std::shared_ptr<Dynamics<SystemDoF, ControlDoF>> m_dynamics;

    /// Keeps track of individual cumulative cost of dynamics simulation rollouts.
    std::shared_ptr<Cost<SystemDoF, ControlDoF>> m_cost;

    ///  The time increment passed to the dynamics simulation when performing rollouts, in seconds.
    const double m_rollout_time_delta;

    /// The duration of time of each rollout, in seconds.
    const double m_rollout_duration;

    /// The number of rollouts to perform on each time.
    const std::size_t m_rollouts;

    /// The control parameters applied at each step in the rollouts.
    Eigen::MatrixXd m_controls;

    /// The evolution of states for each rollout.
    Eigen::MatrixXd m_states;

    /// The cost of each rollout.
    Eigen::MatrixXd m_costs; 

    /// The optimal control trajectory.
    Eigen::MatrixXd m_optimal_trajectory;
};

template<typename DynamicsType, typename CostType>
MPPI<DynamicsType, CostType>::MPPI(
    const std::shared_ptr<DynamicsType> &dynamics,
    const std::shared_ptr<CostType> &cost,
    const State &state,
    double rollout_time_delta,
    double rollout_duration,
    std::size_t rollouts
) : m_dynamics(dynamics)
  , m_cost(cost)
  , m_rollouts(rollouts)
  , m_rollout_time_delta(rollout_time_delta)
  , m_rollout_duration(rollout_duration)
  , m_controls(rollouts * SystemDoF, )
  , m_states()
  , m_costs()
  , m_optimal_trajectory()
{
    m_dynamics->set(state);
    m_cost->clear();
}

template<typename DynamicsType, typename CostType>
void MPPI<DynamicsType, CostType>::generate_control_trajectory(
    const Vector<SystemDoF> &state
) {

    // Rollout
    for (int i = 0; i < m_rollouts; i++) {
        for (double t = 0)
    }

    // Optimise.

    m_optimal_trajectory;
}

} // namespace mpxx
