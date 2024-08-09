#include "test/case/circle.hpp"

#include "frankaridgeback/dynamics.hpp"
#include "frankaridgeback/objective/assisted_manipulation.hpp"

std::unique_ptr<Test> Circle::create()
{
    auto test = std::make_unique<Circle>();

    auto simulator = Simulator::create({
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    });

    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    auto cwd = std::filesystem::current_path();
    std::string urdf = (cwd / "model/robot.urdf").string();

    FrankaRidgebackActor::Configuration configuration {
        .mppi = {
            .dynamics = FrankaRidgeback::Dynamics::create(),
            .cost = AssistedManipulation::create({
                .model = {
                    .filename = urdf,
                    .end_effector_frame = "panda_grasp_joint"
                },
                .enable_joint_limits = true,
                .lower_joint_limit = AssistedManipulation::s_default_lower_joint_limits,
                .upper_joint_limit = AssistedManipulation::s_default_upper_joint_limits
            }),
            .filter = std::nullopt,
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
        .urdf_filename = urdf,
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
    };

    if (!configuration.mppi.cost) {
        std::cerr << "failed to create mppi cost" << std::endl;
        return nullptr;
    }

    if (!configuration.mppi.dynamics) {
        std::cerr << "failed to create mppi dynamics" << std::endl;
        return nullptr;
    }

    auto robot = FrankaRidgebackActor::create(
        std::move(configuration),
        simulator.get()
    );

    if (!robot) {
        std::cerr << "failed to create FrankaRidgebackActor actor" << std::endl;
        return nullptr;
    }

    auto circle_actor = CircleActor::create(CircleActor::Configuration{
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
            },
            .robot = robot.get(),
        },
        simulator.get()
    );

    simulator->add_actor(robot);
    simulator->add_actor(circle_actor);

    auto mppi_logger = logger::MPPI::create(logger::MPPI::Configuration{
        .folder = cwd / "mppi",
        .state_dof = FrankaRidgeback::DoF::STATE,
        .control_dof = FrankaRidgeback::DoF::CONTROL,
        .rollouts = (unsigned int)robot->get_trajectory().get_rollout_count()
    });

    if (!mppi_logger) {
        return nullptr;
    }

    auto pid_logger = logger::PID::create(logger::PID::Configuration{
        .folder = "pid",
        .reference_dof = 3,
        .control_dof = 3
    });

    test->m_simulator = std::move(simulator);
    test->m_robot = std::move(robot);
    test->m_actor = std::move(circle_actor);
    test->m_mppi_logger = std::move(mppi_logger);
    test->m_pid_logger = std::move(pid_logger);

    return test; 
}

bool Circle::run()
{
    // while (m_simulator->get_time() < 30) {
    while (1) {
        raisim::TimedLoop(m_simulator->get_time_step() * 1e6);
        m_simulator->step();
        m_mppi_logger->log(m_robot->get_trajectory());
        m_pid_logger->log(m_actor->get_pid());
    }

    return true;
}
