#pragma once

#include <Eigen/Eigen>

#include "mppi.hpp"

namespace controller {

using namespace mppi;

/**
 * @brief Configuration of a controller.
 */
struct Configuration {

    /// Pointer to the dynamics.
    std::unique_ptr<Dynamics> dynamics;

    /// Pointer to the objective function.
    std::unique_ptr<Cost> cost;

    /// Configuration of the mppi trajectory generator.
    ::mppi::Configuration trajectory;

    /// The initial state of the system.
    Eigen::VectorXd initial_state;
};

/**
 * @brief A controller using MPPI trajectory generation.
 * 
 * Augments mppi trajectory generation with additional logic and logging.
 */
class Controller
{
public:

    /**
     * @brief Create a new controller.
     * 
     * @param configuration The controller configuration.
     * @return A pointer to the controller on success, or nullptr on failure.
     */
    static std::unique_ptr<Controller> create(Configuration &&configuration);

    /**
     * @brief Update the trajectory from a state and a time.
     * 
     * @param state The current state of the dynamics.
     * @param time The current time in seconds.
     */
    void update(const Eigen::VectorXd &state, double time);

    /**
     * @brief Evaluate the current optimal control trajectory at a given time.
     * 
     * @param control Reference to the control vector to fill.
     * @param t The time to evaluate the control trajectory.
     */
    void get(Eigen::VectorXd &control, double time) const;

private:

    /**
     * @brief Construct a new Controller object
     * 
     * @param dynamics Pointer to the dynamics calculator.
     * @param cost Pointer to the cost calculator.
     * @param trajectory Pointer to the trajectory.
     */
    Controller(
        std::unique_ptr<Dynamics> dynamics,
        std::unique_ptr<Cost> cost,
        std::unique_ptr<Trajectory> trajectory
    );

    /// Pointer to the dynamics calculator.
    std::unique_ptr<Dynamics> m_dynamics;

    /// Pointer to the cost calculator.
    std::unique_ptr<Cost> m_cost;

    /// Pointer to the trajectory generator.
    std::unique_ptr<mppi::Trajectory> m_trajectory;
};

} // namespace Controller
