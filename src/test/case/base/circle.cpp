#include "test/case/base/circle.hpp"

#include "frankaridgeback/dynamics.hpp"

Circle::Configuration Circle::DEFAULT_CONFIGURATION {
    .folder = "circle",
    .duration = 30,
    .simulator = {
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    },
    .objective = {
        .model = {
            .filename = "",
            .end_effector_frame = "panda_grasp_joint"
        },
        .enable_joint_limit = true,
        .enable_reach_limit = false,
        .enable_maximise_manipulability = false,
        .enable_minimise_power = false,
        .enable_variable_damping = false,
        .lower_joint_limit = AssistedManipulation::s_default_lower_joint_limits,
        .upper_joint_limit = AssistedManipulation::s_default_upper_joint_limits
    },
    .frankaridgeback_actor = {
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
                0.1, 0.1, 0.2, // base
                10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
                0.0, 0.0 // gripper
            }.asDiagonal(),
            .control_bound = false,
            .control_min = FrankaRidgeback::Control{
                -0, -0, -0, // base
                -0, -0, -0, -0, -0, -0, -0, //arm
                -0, -0 // gripper
            },
            .control_max = FrankaRidgeback::Control{
                0, 0, 0, // base
                0, 0, 0, 0, 0, 0, 0, // arm
                0, 0 // gripper
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
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // arm
            100.0, 100.0 //gripper
        },
        .differential_gain = FrankaRidgeback::Control{
            1000.0, 1000.0, 1.0, // base
            10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
            50.0, 50.0 // gripper
        },
    },
    .circle_actor = {
        .rotating_point = {
            .origin = Eigen::Vector3d(0.75, 0.75, 0.75),
            .axis = Eigen::Vector3d(0.0, 0.0, 1.0),
            .radius = 0.25,
            .angular_velocity = M_PI / 3
        },
        .pid = {
            .state_dof = 3,
            .control_dof = 3,
            .kp = Eigen::Vector3d(500.0, 500.0, 500.0),
            .kd = Eigen::Vector3d(50.0, 50.0, 50.0),
            .ki = Eigen::Vector3d(0.0, 0.0, 0.0),
            .minimum = Eigen::Vector3d(-10000.0, -10000.0, -10000.0),
            .maximum = Eigen::Vector3d(10000.0, 10000.0, 10000.0),
            .reference = Eigen::Vector3d::Zero()
        }
    },
    .mppi_logger {
        .folder = "",
        .state_dof = FrankaRidgeback::DoF::STATE,
        .control_dof = FrankaRidgeback::DoF::CONTROL,
        .rollouts = 0
    },
    .pid_logger = {
        .folder = "",
        .reference_dof = 3,
        .control_dof = 3
    }
};  

std::unique_ptr<Test> Circle::create(Options &options)
{
    Configuration configuration = DEFAULT_CONFIGURATION;
    configuration.duration = options.duration;
    configuration.folder = options.folder;

    // If configuration overrides were provided, apply them based on the json
    // patch specification.
    try {
        if (!options.patch.is_null()) {
            json json_configuration = DEFAULT_CONFIGURATION;
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

std::unique_ptr<Test> Circle::create(const Configuration &configuration)
{
    if (configuration.duration <= 0.0) {
        std::cerr << "test duration <= 0" << std::endl;
        return nullptr;
    }

    if (configuration.folder.empty()) {
        std::cerr << "output folder path is empty" << std::endl;
        return nullptr;
    }

    auto simulator = Simulator::create(configuration.simulator);
    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    auto cost_configuration = configuration.objective;
    if (cost_configuration.model.filename.empty())
        cost_configuration.model.filename = FrankaRidgeback::Model::find_path().string();

    auto cost = AssistedManipulation::create(cost_configuration);
    if (!cost) {
        std::cerr << "failed to create mppi cost" << std::endl;
        return nullptr;
    }

    auto dynamics = FrankaRidgeback::Dynamics::create();
    if (!dynamics) {
        std::cerr << "failed to create mppi dynamics" << std::endl;
        return nullptr;
    }

    auto robot_configuration = configuration.frankaridgeback_actor;
    if (robot_configuration.urdf_filename.empty())
        robot_configuration.urdf_filename = FrankaRidgeback::Model::find_path().string();

    auto robot = FrankaRidgebackActor::create(
        robot_configuration,
        simulator.get(),
        std::move(dynamics),
        std::move(cost)
    );

    if (!robot) {
        std::cerr << "failed to create FrankaRidgebackActor actor" << std::endl;
        return nullptr;
    }

    auto circle_actor = CircleActor::create(
        configuration.circle_actor,
        simulator.get(),
        robot.get()
    );

    if (!circle_actor) {
        std::cerr << "failed to create circle actor" << std::endl;
        return nullptr;
    }

    simulator->add_actor(robot);
    simulator->add_actor(circle_actor);

    // Update mppi logger configuration.
    logger::MPPI::Configuration mppi = configuration.mppi_logger;
    mppi.rollouts = robot->get_trajectory().get_rollout_count();
    if (mppi.folder.empty())
        mppi.folder = configuration.folder / "mppi";

    auto mppi_logger = logger::MPPI::create(mppi);
    if (!mppi_logger) {
        std::cerr << "failed to create mppi logger" << std::endl;
        return nullptr;
    }

    // Update pid logger configuration.
    logger::PID::Configuration pid = configuration.pid_logger;
    if (pid.folder.empty())
        pid.folder = configuration.folder / "pid";

    auto pid_logger = logger::PID::create(pid);
    if (!pid_logger) {
        std::cerr << "failed to create pid logger" << std::endl;
        return nullptr;
    }

    // Log the configuration used in the test.
    {
        auto file = logger::File::create(configuration.folder / "configuration.json");
        if (!file)
            return nullptr;
        file->get_stream() << ((json)configuration).dump(4);
    }

    auto test = std::make_unique<Circle>();

    test->m_duration = configuration.duration;
    test->m_simulator = std::move(simulator);
    test->m_robot = std::move(robot);
    test->m_actor = std::move(circle_actor);
    test->m_mppi_logger = std::move(mppi_logger);
    test->m_pid_logger = std::move(pid_logger);

    return test; 
}

bool Circle::run()
{
    while (m_simulator->get_time() < m_duration) {
        // raisim::TimedLoop(m_simulator->get_time_step() * 1e6);
        m_simulator->step();
        m_mppi_logger->log(m_robot->get_trajectory());
        m_pid_logger->log(m_actor->get_pid());
    }

    return true;
}
