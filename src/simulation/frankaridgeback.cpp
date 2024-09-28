#include "simulation/frankaridgeback.hpp"

namespace FrankaRidgeback {

std::shared_ptr<Actor> Actor::create(
    Configuration configuration,
    Simulator *simulator,
    std::unique_ptr<mppi::Cost> &&cost,
    std::unique_ptr<Forecast::Handle> &&wrench_forecast, 
    std::unique_ptr<mppi::Filter> &&filter
) {
    using namespace controller;

    std::unique_ptr<Forecast::Handle> mppi_forecast = nullptr;
    if (wrench_forecast)
        mppi_forecast = wrench_forecast->copy();

    // Create the dynamics to be simulated for the frankaridgeback robot.
    auto adaptor = SimulatorAdaptor::create(
        configuration.dynamics,
        simulator,
        std::move(mppi_forecast)
    );
    if (!adaptor) {
        std::cerr << "failed to create simulator actor dynamics" << std::endl;
        return nullptr;
    }

    std::unique_ptr<mppi::Dynamics> mppi_dynamics = nullptr;

    // Create the dynamics for the mppi controller.
    if (configuration.mppi_type == Type::RAISIM) {
        if (!configuration.mppi_raisim) {
            std::cerr << "no mppi raisim dynamics configuration provided" << std::endl;
            return nullptr;
        }
        mppi_dynamics = RaisimDynamics::create(
            *configuration.mppi_raisim,
            std::move(mppi_forecast)
        );
    }
    else if (configuration.mppi_type == Type::PINOCCHIO) {
        if (!configuration.mppi_pinocchio) {
            std::cerr << "no mppi pinocchio dynamics configuration provided" << std::endl;
            return nullptr;
        }
        mppi_dynamics = PinocchioDynamics::create(
            *configuration.mppi_pinocchio,
            std::move(mppi_forecast)
        );
    }
    else {
        std::cerr << "unrecognised mppi dynamics type " << configuration.mppi_type << std::endl;
        return nullptr;
    }

    // Verify the dynamics was created successfully.
    if (!mppi_dynamics) {
        std::cerr << "failed to create mppi dynamics" << std::endl;
        return nullptr;
    }

    auto controller = mppi::Trajectory::create(
        configuration.mppi,
        std::move(mppi_dynamics),
        std::move(cost),
        std::move(filter)
    );
    if (!controller) {
        std::cerr << "failed to create Actor mppi" << std::endl;
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
            std::move(configuration),
            std::move(adaptor),
            std::move(controller),
            controller_countdown_max
        )
    );
}

Actor::Actor(
    Configuration &&configuration,
    std::unique_ptr<SimulatorAdaptor> &&dynamics,
    std::unique_ptr<mppi::Trajectory> &&controller,
    std::int64_t controller_countdown_max
) : m_configuration(std::move(configuration))
  , m_dynamics(std::move(dynamics))
  , m_controller(std::move(controller))
  , m_trajectory_countdown(0) // Update on first step.
  , m_trajectory_countdown_max(controller_countdown_max)
  , m_control(FrankaRidgeback::Control::Zero())
{}

void Actor::act(Simulator *simulator)
{
    using namespace FrankaRidgeback;

    // Update the controller every couple of time steps, depending on the
    // controller update rate.
    if (--m_trajectory_countdown <= 0) {
        m_trajectory_countdown = m_trajectory_countdown_max;

        for (unsigned int i = 0; i < m_configuration.controller_substeps; i++) {
            m_controller->update(
                m_dynamics->get_dynamics()->get_state(),
                simulator->get_time()
            );
        }
    }

    // Get the controls to apply to the dynamics.
    m_controller->get(m_control, simulator->get_time());
    m_dynamics->act(m_control, simulator->get_time_step());
}

void Actor::update(Simulator *simulator)
{
    m_dynamics->update();
}

} // namespace FrankaRidgeback
