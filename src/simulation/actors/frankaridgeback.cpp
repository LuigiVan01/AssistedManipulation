#include "simulation/actors/frankaridgeback.hpp"

namespace FrankaRidgeback {

std::shared_ptr<Actor> Actor::create(
    const Configuration &configuration,
    Simulator *simulator,
    std::unique_ptr<mppi::Cost> &&cost,
    std::unique_ptr<Forecast::Handle> &&force_forecast, 
    std::unique_ptr<mppi::Filter> &&filter
) {
    using namespace controller;

    auto dynamics = FrankaRidgeback::RaisimDynamics::create(
        configuration.dynamics,
        std::move(force_forecast)
    );
    if (!dynamics) {
        std::cerr << "failed to create frankaridgeback raisim dynamics" << std::endl;
        return nullptr;
    }

    auto controller = mppi::Trajectory::create(
        configuration.mppi,
        std::move(dynamics->copy()),
        std::move(cost),
        std::move(filter)
    );
    if (!controller) {
        std::cerr << "failed to create Actor mppi" << std::endl;
        return nullptr;
    }

    auto visual = simulator->get_server().addVisualArticulatedSystem(
        "frankaridgeback",
        configuration.dynamics.filename
    );
    if (!visual) {
        std::cerr << "failed to create frankaridgeback actor visual" << std::endl;
        return nullptr;
    }

    // The rate of updating 
    std::int64_t controller_countdown_max = (std::int64_t)(
        configuration.controller_rate / simulator->get_time_step()
    );

    // There must be at least one controller update.
    if (configuration.controller_substeps < 1) {
        std::cerr << "frankaridgeback actor substeps less than zero" << std::endl;
        return nullptr;
    }

    return std::shared_ptr<Actor>(
        new Actor(
            configuration,
            std::move(dynamics),
            std::move(controller),
            visual,
            controller_countdown_max
        )
    );
}

Actor::Actor(
    const Configuration &configuration,
    std::unique_ptr<RaisimDynamics> &&dynamics,
    std::unique_ptr<mppi::Trajectory> &&controller,
    raisim::ArticulatedSystemVisual *visual,
    std::int64_t controller_countdown_max
) : m_configuration(std::move(configuration))
  , m_dynamics(std::move(dynamics))
  , m_trajectory(std::move(controller))
  , m_visual(visual)
  , m_trajectory_countdown(0) // Update on first step.
  , m_trajectory_countdown_max(controller_countdown_max)
  , m_state(FrankaRidgeback::State::Zero())
{}

void Actor::act(Simulator *simulator)
{
    using namespace FrankaRidgeback;

    // Update the controller every couple of time steps, depending on the
    // controller update rate.
    if (--m_trajectory_countdown <= 0) {
        m_trajectory_countdown = m_trajectory_countdown_max;

        for (unsigned int i = 0; i < m_configuration.controller_substeps; i++)
            m_trajectory->update(m_state, simulator->get_time());
    }

    // Get the controls to apply to the dynamics.
    m_trajectory->get(m_control, simulator->get_time());
    m_state = m_dynamics->step(m_control, simulator->get_time_step());
}

void Actor::update(Simulator *simulator)
{
    m_visual->setGeneralizedCoordinate(m_state.position());
}

} // namespace FrankaRidgeback
