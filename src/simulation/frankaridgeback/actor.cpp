#include "simulation/frankaridgeback/actor.hpp"

namespace FrankaRidgeback {

std::shared_ptr<Actor> Actor::create(
    Configuration configuration,
    Simulator *simulator
) {
    using namespace controller;

    // Create an adaptor between the configured dactor model dynamics and the
    // simulator. The adaptor adds additional functionality to better integrate
    // with the simulation, such as visualisation.
    auto dynamics = ActorDynamics::create(configuration.dynamics, simulator);
    if (!dynamics) {
        std::cerr << "failed to create simulated actor dynamics" << std::endl;
        return nullptr;
    }

    std::unique_ptr<mppi::Cost> objective;

    switch (configuration.objective.type)
    {
        case Configuration::Objective::Type::ASSISTED_MANIPULATION: {
            if (!configuration.objective.assisted_manipulation) {
                std::cerr << "assisted manipulation objective selected with no configuration" << std::endl;
                return nullptr;
            }

            objective = AssistedManipulation::create(
                *configuration.objective.assisted_manipulation
            );
            break;
        }
        case Configuration::Objective::Type::TRACK_POINT: {
            if (!configuration.objective.track_point) {
                std::cerr << "assisted manipulation objective selected with no configuration" << std::endl;
                return nullptr;
            }

            objective = TrackPoint::create(*configuration.objective.track_point);
            break;
        }
        default: {
            std::cerr << "unknown objective type " << configuration.objective.type << " provided" << std::endl;
            return nullptr;
        }
    }

    if (!objective) {
        std::cerr << "failed to create mppi cost" << std::endl;
        return nullptr;
    }

    std::unique_ptr<DynamicsForecast> forecast = nullptr;
    std::unique_ptr<DynamicsForecast::Handle> forecast_handle = nullptr;

    // If the actor should forecast the dynamics before a controller update then
    // instantiate the dynamics and forecast used to rollout the forecast.
    if (configuration.forecast) {
        auto forecast_dynamics = SimulatorDynamics::create(
            configuration.forecast->dynamics,
            nullptr
        );
        if (!forecast_dynamics) {
            std::cerr << "failed to create dynamics for forecasting" << std::endl;
            return nullptr;
        }

        forecast = DynamicsForecast::create(
            configuration.forecast->configuration,
            std::move(forecast_dynamics)
        );
        if (!forecast) {
            std::cerr << "failed to create dynamics forecast instance" << std::endl;
            return nullptr;
        }

        forecast_handle = forecast->create_handle();
    }

    // Instantiate the dynamics used in the MPPI trajectory generator. Don't
    // pass the simulator; the raisim simulation will use its own world. This
    // instance is copied to all the MPPI threads to perform rollouts.
    auto mppi_dynamics = SimulatorDynamics::create(
        configuration.mppi.dynamics,
        std::move(forecast_handle),
        nullptr
    );
    if (!mppi_dynamics) {
        std::cerr << "failed to create mppi dynamics" << std::endl;
        return nullptr;
    }

    // Create the trajectory generator.
    auto controller = mppi::Trajectory::create(
        configuration.mppi.configuration,
        std::move(mppi_dynamics),
        std::move(objective),
        nullptr
    );
    if (!controller) {
        std::cerr << "failed to create Actor mppi" << std::endl;
        return nullptr;
    }

    // The rate of updating the trajectory generator.
    std::int64_t controller_countdown_max = (std::int64_t)(
        configuration.controller_rate / simulator->get_time_step()
    );

    // The rate of updating the forecast with observed state. Only rolled out
    // on controller update.
    std::int64_t forecast_countdown_max = (std::int64_t)(
        configuration.forecast_rate / simulator->get_time_step()
    );

    // There must be at least one controller update.
    if (configuration.controller_substeps < 1) {
        std::cerr << "frankaridgeback actor substeps less than zero" << std::endl;
        return nullptr;
    }

    return std::shared_ptr<Actor>(
        new Actor(
            std::move(configuration),
            std::move(dynamics),
            std::move(controller),
            std::move(forecast),
            controller_countdown_max,
            forecast_countdown_max
        )
    );
}

Actor::Actor(
    Configuration &&configuration,
    std::unique_ptr<ActorDynamics> &&dynamics,
    std::unique_ptr<mppi::Trajectory> &&controller,
    std::unique_ptr<DynamicsForecast> &&forecast,
    std::int64_t controller_countdown_max,
    std::int64_t forecast_countdown_max
) : m_configuration(std::move(configuration))
  , m_dynamics(std::move(dynamics))
  , m_forecast(std::move(forecast))
  , m_controller(std::move(controller))
  , m_trajectory_countdown(0) // Update on first step.
  , m_trajectory_countdown_max(controller_countdown_max)
  , m_forecast_countdown(0)
  , m_forecast_countdown_max(forecast_countdown_max)
  , m_control(FrankaRidgeback::Control::Zero())
{}

void Actor::add_end_effector_wrench(Vector6d wrench, double time)
{
    m_dynamics->get_dynamics()->add_end_effector_simulated_wrench(wrench);

    if (m_forecast && m_forecast_countdown <= 0) {
        //! The forecast of the wrench given a wrench measurement is performed only here
        //! This is because the wrench is applied to the end effector in this method 
        m_forecast->observe_wrench(wrench, time);
        m_forecast_countdown = m_forecast_countdown_max;
    }
}

void Actor::act(Simulator *simulator)
{
    using namespace FrankaRidgeback;

    // The controller is updating at a lower frequency than the simulation rate
    // The m_trajectory_countdown goes to zero when it is time to forecast and update the controller
    if (--m_trajectory_countdown <= 0) {
        m_trajectory_countdown = m_trajectory_countdown_max; // Reset counter

        // Forecast the wrench
        if (m_forecast) {
            m_forecast->forecast(
                m_dynamics->get_dynamics()->get_state(),
                simulator->get_time()
            );
        }

        // Update the controller m_configuration.controller_substeps times 
        //! I don t know why it would make sense to update the controller more times in a row. 
        //! In the default configuration m_configuration.controller_substeps = 1
        for (unsigned int i = 0; i < m_configuration.controller_substeps; i++) {
            m_controller->update(
                m_dynamics->get_dynamics()->get_state(),
                simulator->get_time()
            );
        }
    }

    if (m_forecast) {
        if (m_forecast_countdown-- != m_forecast_countdown_max) {
            m_forecast->observe_time(simulator->get_time());
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
