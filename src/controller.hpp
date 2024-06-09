#pragma once

#include "mppi.hpp"

template<typename Dynamics, typename Cost>
class Controller
{
public:

    std::unique_ptr<Controller<Dynamics, Cost>> create();

private:

    /**
     * @brief Construct a new Controller object
     * 
     * @param configuration The mppi trajectory generation configuration.
     * @param dynamics Pointer to the dynamics calculator.
     * @param cost Pointer to the cost calculator.
     * @param trajectory Pointer to the trajectory.
     */
    Controller(
        mppi::Configuration &&configuration,
        std::shared_ptr<Dynamics> &&dynamics,
        std::shared_ptr<Cost> &&cost,
        std::unique<mppi::Trajectory<Dynamics, Cost>> &&trajectory
    );

    /// The configuration of the controller.
    mppi::Configuration m_configuration;

    /// Pointer to the dynamics calculator.
    std::shared_ptr<Dynamics> m_dynamics;

    /// Pointer to the cost calculator.
    std::shared_ptr<Cost> m_cost;

    /// Pointer to the trajectory generator.
    std::unique_ptr<mppi::Trajectory<Dynamics, Cost>> m_trajectory;
};

template<typename Dynamics, typename Cost>
std::unique_ptr<Controller<Dynamics, Cost>> create()
{
    return nullptr
}
