#include "controller.hpp"

namespace controller {

std::unique_ptr<Controller> Controller::create(Configuration &&config)
{
    // Must create the dynamics before this call.
    if (!config.dynamics) {
        std::cerr << "controller dynamics nullptr" << std::endl;
        return nullptr;
    }

    // Must create the cost before this call.
    if (!config.cost) {
        std::cerr << "controller cost nullptr" << std::endl;
        return nullptr;
    }

    auto trajectory = Trajectory::create(
        config.dynamics.get(),
        config.cost.get(),
        config.trajectory,
        config.initial_state
    );

    if (!trajectory) {
        std::cerr << "controller could not create trajectory" << std::endl;
        return nullptr;
    }

    return std::unique_ptr<Controller>(
        new Controller(
            std::move(config.dynamics),
            std::move(config.cost),
            std::move(trajectory)
        )
    );
}

Controller::Controller(
    std::unique_ptr<Dynamics> dynamics,
    std::unique_ptr<Cost> cost,
    std::unique_ptr<Trajectory> trajectory
) : m_dynamics(std::move(dynamics))
  , m_cost(std::move(cost))
  , m_trajectory(std::move(trajectory))
{}

} // namespace controller
