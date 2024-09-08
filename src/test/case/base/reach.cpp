#include "test/case/base/reach.hpp"

#include "logging/file.hpp"

ReachForPoint::Configuration ReachForPoint::DEFAULT_CONFIGIURATION {
    .folder = "reach",
    .simulator = {
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    },
    .dynamics = {
        .model = {
            .filename = "",
            .end_effector_frame = "panda_grasp"
        }
    },
    .objective = {
        .point = Eigen::Vector3d(1.0, 1.0, 1.0),
    },
    .actor = {
        .mppi = {
            .initial_state = FrankaRidgeback::State::Zero(),
            .rollouts = 20,
            .keep_best_rollouts = 10,
            .time_step = 0.1,
            .horison = 1.0,
            .gradient_step = 1.0,
            .cost_scale = 10.0,
            .cost_discount_factor = 1.0,
            .covariance = FrankaRidgeback::Control{
                0.0, 0.0, 0.2, // base
                10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 // arm
            }.asDiagonal(),
            .control_bound = false,
            .control_min = FrankaRidgeback::Control{
                -0.2, -0.2, -0.2, // base
                -5.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 //arm
            },
            .control_max = FrankaRidgeback::Control{
                0.2, 0.2, 0.2, // base
                5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 // arm
            },
            .control_default = FrankaRidgeback::Control::Zero(),
            .smoothing = std::nullopt,
            .threads = 12
        },
        .controller_rate = 0.3,
        .controller_substeps = 10,
        .urdf_filename = "",
        .end_effector_frame = "panda_grasp_joint",
        .initial_state = FrankaRidgeback::State::Zero(),
        .proportional_gain = FrankaRidgeback::Control{
            0.0, 0.0, 0.0, // base
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 // arm
        },
        .differential_gain = FrankaRidgeback::Control{
            1000.0, 1000.0, 1.0, // base
            10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 // arm
        },
    },
    .logger = {
        .folder = "",
        .state_dof = FrankaRidgeback::DoF::STATE,
        .control_dof = FrankaRidgeback::DoF::CONTROL,
        .rollouts = 0
    }
};

std::unique_ptr<Test> ReachForPoint::create(Options &options)
{
    Configuration configuration = DEFAULT_CONFIGIURATION;
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
        std::cerr << "configuration was " << ((json)DEFAULT_CONFIGIURATION).dump(4) << std::endl;
        std::cerr << "patch was " << options.patch.dump(4) << std::endl;
        return nullptr;
    }

    return create(configuration);
}

std::unique_ptr<Test> ReachForPoint::create(const Configuration &configuration)
{
    std::unique_ptr<Simulator> simulator = Simulator::create(configuration.simulator);
    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    auto cost = TrackPoint::create(configuration.objective);
    if (!cost) {
        std::cerr << "failed to create mppi cost" << std::endl;
        return nullptr;
    }

    auto dynamics_configuration = configuration.dynamics;
    if (dynamics_configuration.model.filename.empty())
        dynamics_configuration.model.filename = FrankaRidgeback::Model::find_path().string();

    auto dynamics = FrankaRidgeback::Dynamics::create(dynamics_configuration);
    if (!dynamics) {
        std::cerr << "failed to create mppi dynamics" << std::endl;
        return nullptr;
    }

    auto actor_configuration = configuration.actor;
    if (actor_configuration.urdf_filename.empty())
        actor_configuration.urdf_filename = FrankaRidgeback::Model::find_path().string();

    auto robot = FrankaRidgebackActor::create(
        actor_configuration,
        simulator.get(),
        std::move(dynamics),
        std::move(cost)
    );

    if (!robot) {
        std::cerr << "failed to create frankaridgeback actor" << std::endl;
        return nullptr;
    }

    simulator->add_actor(robot);

    // Override rollout count of logger.
    logger::MPPI::Configuration log = configuration.logger;
    log.rollouts = robot->get_trajectory().get_rollout_count();
    if (log.folder.empty())
        log.folder = configuration.folder / "pid";

    auto logger = logger::MPPI::create(log);
    if (!logger) {
        return nullptr;
    }

    // Log the configuration used in the test.
    {
        auto file = logger::File::create(configuration.folder / "configuration.json");
        if (!file)
            return nullptr;
        file->get_stream() << ((json)configuration).dump(4);
    }

    auto test = std::unique_ptr<ReachForPoint>(new ReachForPoint());

    test->m_duration = configuration.duration;
    test->m_simulator = std::move(simulator);
    test->m_robot = std::move(robot);
    test->m_mppi_logger = std::move(logger);

    return test;
}

bool ReachForPoint::run()
{
    while (m_simulator->get_time() < 30) {
        // raisim::TimedLoop(m_simulator->get_time_step() * 1e6);
        m_simulator->step();
        m_mppi_logger->log(m_robot->get_trajectory());
    }

    return true;
}