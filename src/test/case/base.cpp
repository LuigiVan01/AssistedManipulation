#include "test/case/base.hpp"

#include "logging/file.hpp"

const BaseTest::Configuration BaseTest::DEFAULT_CONFIGURATION {
    .folder = "default",
    .duration = 30.0,
    .simulator = {
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    },
    .actor = {
        .mppi = {
            .configuration = {
                .initial_state = make_state(FrankaRidgeback::Preset::HUDDLED_10J),
                .rollouts = 20,
                .keep_best_rollouts = 15,
                .time_step = 0.01,
                .horison = 0.25,
                .gradient_step = 1.0,
                .cost_scale = 10.0,
                .cost_discount_factor = 1.0,
                .covariance = FrankaRidgeback::Control{
                    0.1, 0.1, 0.2, // base
                    7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, // arm
                    0.0, 0.0 // gripper
                }.asDiagonal(),
                .control_bound = false,
                .control_min = FrankaRidgeback::Control{
                    -0.2, -0.2, -0.2, // base
                    -5.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, //arm
                    -0.05, -0.05 // gripper
                },
                .control_max = FrankaRidgeback::Control{
                    0.2, 0.2, 0.2, // base
                    5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, // arm
                    0.05, 0.05 // gripper
                },
                .control_default = FrankaRidgeback::Control::Zero(),
                .smoothing = std::nullopt,
                // .smoothing = mppi::Configuration::Smoothing {
                //     .window = 10,
                //     .order = 1
                // },
                .threads = 12
            },
            .dynamics = {
                .type = FrankaRidgeback::SimulatorDynamics::Configuration::Type::RAISIM,
                .raisim = FrankaRidgeback::RaisimDynamics::DEFAULT_CONFIGURATION,
                .pinocchio = FrankaRidgeback::PinocchioDynamics::DEFAULT_CONFIGURATION
            }
        },
        .dynamics = {
            .type = FrankaRidgeback::SimulatorDynamics::Configuration::Type::RAISIM,
            .raisim = FrankaRidgeback::RaisimDynamics::DEFAULT_CONFIGURATION,
            .pinocchio = FrankaRidgeback::PinocchioDynamics::DEFAULT_CONFIGURATION
        },
        .objective = {
            .type = FrankaRidgeback::Actor::Configuration::Objective::Type::ASSISTED_MANIPULATION,
            .assisted_manipulation = FrankaRidgeback::AssistedManipulation::DEFAULT_CONFIGURATION,
            .track_point = FrankaRidgeback::TrackPoint::DEFAULT_CONFIGURATION
        },
        .forecast = FrankaRidgeback::Actor::Configuration::Forecast {
            .configuration = {
                .time_step = 0.01,
                .horison = 1.0,
                .end_effector_wrench_forecast = {
                    .type = Forecast::Configuration::Type::LOCF,
                    .locf = LOCFForecast::Configuration {
                        .observation = Vector6d::Zero()
                    },
                    .average = AverageForecast::Configuration {
                        .states = 6,
                        .window = 1.0
                    },
                    .kalman = std::nullopt
                }
            },
            .dynamics = {
                .type = FrankaRidgeback::SimulatorDynamics::Configuration::Type::RAISIM,
                .raisim = FrankaRidgeback::RaisimDynamics::DEFAULT_CONFIGURATION,
                .pinocchio = FrankaRidgeback::PinocchioDynamics::DEFAULT_CONFIGURATION
            }
        },
        .controller_rate = 0.03,
        .controller_substeps = 1,
        .forecast_rate = 0.15
    },
    .mppi_logger = {
        .folder = "",
        .state_dof = FrankaRidgeback::DoF::STATE,
        .control_dof = FrankaRidgeback::DoF::CONTROL,
        .rollouts = 0
    },
    .dynamics_logger = {
        .folder = "",
        .log_end_effector_position = true,
        .log_end_effector_velocity = true,
        .log_end_effector_acceleration = true,
        .log_power = true,
        .log_tank_energy = true,
    },
    .forecast_logger = {
        .folder = "",
        .log_end_effector_position = true,
        .log_end_effector_velocity = true,
        .log_end_effector_acceleration = true,
        .log_power = true,
        .log_tank_energy = true,
        .log_wrench = true
    }
};

std::unique_ptr<BaseTest> BaseTest::create(Options &options)
{
    Configuration configuration = DEFAULT_CONFIGURATION;
    configuration.duration = options.duration;
    configuration.folder = options.folder;

    // If configuration overrides were provided, apply them based on the json
    // patch specification.
    try {
        if (!options.patch.is_null()) {
            json json_configuration = configuration;
            json_configuration.merge_patch(options.patch);
            configuration = json_configuration;
        }
    }
    catch (const json::exception &err) {
        std::cerr << "error when patching json configuration: " << err.what() << std::endl;
        std::cerr << "configuration was " << ((json)DEFAULT_CONFIGURATION).dump(4) << std::endl;
        std::cerr << "patch was " << options.patch.dump(4) << std::endl;
        return nullptr;
    }

    return create(configuration);
}

std::unique_ptr<BaseTest> BaseTest::create(const Configuration &configuration)
{
    std::unique_ptr<Simulator> simulator = Simulator::create(configuration.simulator);
    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    auto frankaridgeback = FrankaRidgeback::Actor::create(
        configuration.actor,
        simulator.get()
    );

    if (!frankaridgeback) {
        std::cerr << "failed to create frankaridgeback actor" << std::endl;
        return nullptr;
    }

    simulator->add_actor(frankaridgeback);

    // Override rollout count of logger.
    auto mppi_log_configuration = configuration.mppi_logger;
    auto dynamics_log_configuration = configuration.dynamics_logger;
    auto forecast_log_configuration = configuration.forecast_logger;

    mppi_log_configuration.rollouts = frankaridgeback->get_controller().get_rollout_count();
    mppi_log_configuration.folder = configuration.folder / "mppi";
    dynamics_log_configuration.folder = configuration.folder / "dynamics";
    forecast_log_configuration.folder = configuration.folder / "forecast";

    auto mppi_logger = logger::MPPI::create(mppi_log_configuration);
    if (!mppi_logger) {
        std::cerr << "failed to create mppi logger" << std::endl;
        return nullptr;
    }

    auto dynamics_logger = logger::FrankaRidgebackDynamics::create(dynamics_log_configuration);
    if (!dynamics_logger) {
        std::cerr << "failed to create dynamics logger" << std::endl;
        return nullptr;
    }

    auto forecast_logger = logger::FrankaRidgebackDynamicsForecast::create(forecast_log_configuration);
    if (!forecast_logger) {
        std::cerr << "failed to create forecast logger" << std::endl;
        return nullptr;
    }

    // Log the configuration used in the test.
    {
        auto file = logger::File::create(configuration.folder / "configuration.json");
        if (!file) {
            std::cerr << "failed create configuration file" << std::endl;
            return nullptr;
        }
        file->get_stream() << ((json)configuration).dump(4);
    }

    return std::unique_ptr<BaseTest>(
        new BaseTest(
            configuration.duration,
            std::move(simulator),
            std::move(frankaridgeback),
            std::move(mppi_logger),
            std::move(dynamics_logger),
            std::move(forecast_logger)
        )
    );
}

BaseTest::BaseTest(
    double duration,
    std::unique_ptr<Simulator> &&simulator,
    std::shared_ptr<FrankaRidgeback::Actor> &&frankaridgeback,
    std::unique_ptr<logger::MPPI> &&mppi_logger,
    std::unique_ptr<logger::FrankaRidgebackDynamics> &&dynamics_logger,
    std::unique_ptr<logger::FrankaRidgebackDynamicsForecast> &&forecast_logger
 ) : m_duration(duration)
   , m_simulator(std::move(simulator))
   , m_frankaridgeback(std::move(frankaridgeback))
   , m_mppi_logger(std::move(mppi_logger))
   , m_dynamics_logger(std::move(dynamics_logger))
   , m_forecast_logger(std::move(forecast_logger))
{}

void BaseTest::step()
{
    m_simulator->step();

    auto &forecast = m_frankaridgeback->get_dynamics().get_forecast();
    if (forecast)
        m_forecast_logger->log(*forecast.value()->get());

    m_mppi_logger->log(m_frankaridgeback->get_controller());
    m_dynamics_logger->log(m_simulator->get_time(), m_frankaridgeback->get_dynamics());
}

bool BaseTest::run()
{
    while (m_simulator->get_time() < m_duration) {

        // Stores the current time and a duration. If the duration has not
        // elapsed by the time the destructor is called, waits for the remaining
        // duration. Caps the maximum loop speed to realtime.
        auto delay = raisim::TimedLoop(m_simulator->get_time_step() * 1e6);

        step();
    }

    return true;
}
