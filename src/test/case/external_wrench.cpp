#include "test/case/external_wrench.hpp"

#include "frankaridgeback/dynamics.hpp"

std::unique_ptr<ExternalWrenchTest> ExternalWrenchTest::create(Options &options)
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

std::unique_ptr<ExternalWrenchTest> ExternalWrenchTest::create(Configuration configuration)
{
    if (configuration.duration <= 0.0) {
        std::cerr << "test duration <= 0" << std::endl;
        return nullptr;
    }

    if (configuration.folder.empty()) {
        std::cerr << "output folder path is empty" << std::endl;
        return nullptr;
    }

    configuration.base.duration = configuration.duration;
    configuration.base.folder = configuration.folder;

    auto base = BaseTest::create(configuration.base);
    if (!base) {
        std::cerr << "failed to create base simulation" << std::endl;
        return nullptr;
    }

    if (!configuration.trajectory.position && !configuration.trajectory.orientation) {
        std::cerr << "at least one position or orientation trajectory is required" << std::endl;
        return nullptr;
    }

    // Create the positional part of the trajectory if provided.
    std::unique_ptr<PositionTrajectory> position;
    if (configuration.trajectory.position) {
        position = PositionTrajectory::create(*configuration.trajectory.position);
        if (!position) {
            std::cerr << "failed to create position trajectory" << std::endl;
            return nullptr;
        }
    }

    // Create the orientation part of the trajectory if provided.
    std::unique_ptr<OrientationTrajectory> orientation;
    if (configuration.trajectory.orientation) {
        orientation = OrientationTrajectory::create(*configuration.trajectory.orientation);
        if (!orientation) {
            std::cerr << "failed to create orientation trajectory" << std::endl;
            return nullptr;
        }
    }

    if (configuration.force_pid.n != 3) {
        std::cerr << "the force pid reference trajectory must be dof 3" << std::endl;
        return nullptr;
    }

    std::unique_ptr<controller::PID> force_pid = controller::PID::create(configuration.force_pid);
    if (!force_pid) {
        std::cerr << "failed to create force pid controller for external wrench" << std::endl;
        return nullptr;
    }

    std::unique_ptr<controller::QuaternionPID> torque_pid = controller::QuaternionPID::create(
        configuration.torque_pid
    );
    if (!torque_pid) {
        std::cerr << "failed to create torque pid controller for external wrench" << std::endl;
        return nullptr;
    }

    logger::PID::Configuration force_pid_configuration {
        .folder = configuration.folder / "pid" / "force",
        .reference_dof = 3,
        .control_dof = 3
    };

    auto force_pid_logger = logger::PID::create(force_pid_configuration);
    if (!force_pid_logger) {
        std::cerr << "failed to create force pid logger" << std::endl;
        return nullptr;
    }

    logger::PID::Configuration torque_pid_configuration {
        .folder = configuration.folder / "pid" / "torque",
        .reference_dof = 4,
        .control_dof = 3
    };

    auto torque_pid_logger = logger::PID::create(torque_pid_configuration);
    if (!torque_pid_logger) {
        std::cerr << "failed to create torque pid logger" << std::endl;
        return nullptr;
    }

    // Log the configuration used in the test.
    {
        auto file = logger::File::create(configuration.folder / "configuration.json");
        if (!file)
            return nullptr;
        file->get_stream() << ((json)configuration).dump(4);
    }

    return std::unique_ptr<ExternalWrenchTest>(
        new ExternalWrenchTest(
            std::move(configuration),
            std::move(base),
            std::move(position),
            std::move(orientation),
            std::move(force_pid),
            std::move(torque_pid),
            std::move(force_pid_logger),
            std::move(torque_pid_logger)
        )
    );
}

ExternalWrenchTest::ExternalWrenchTest(
    Configuration &&configuration,
    std::unique_ptr<BaseTest> &&base,
    std::unique_ptr<PositionTrajectory> &&position,
    std::unique_ptr<OrientationTrajectory> &&orientation,
    std::unique_ptr<controller::PID> &&force_pid,
    std::unique_ptr<controller::QuaternionPID> &&torque_pid,
    std::unique_ptr<logger::PID> &&force_pid_logger,
    std::unique_ptr<logger::PID> &&torque_pid_logger
  ) : m_configuration(configuration)
    , m_base(std::move(base))
    , m_position(std::move(position))
    , m_orientation(std::move(orientation))
    , m_force_pid(std::move(force_pid))
    , m_torque_pid(std::move(torque_pid))
    , m_force_pid_logger(std::move(force_pid_logger))
    , m_torque_pid_logger(std::move(torque_pid_logger))
    , m_tracking_sphere(nullptr)
{
    // Create a visual sphere to visualise the current trajectory position.
    if (m_position) {
        m_tracking_sphere = m_base->get_simulator()->get_server().addVisualSphere(
            "tracking_sphere", 0.05
        );
    }
}

bool ExternalWrenchTest::run()
{
    // Optional forecast
    Vector6d wrench = Vector6d::Zero();

    for (;;) {
        double time = m_base->get_simulator()->get_time();

        if (time >= m_configuration.duration)
            break;

        // Stores the current time and a duration. If the duration has not
        // elapsed by the time the destructor is called, waits for the remaining
        // duration. Caps the maximum loop speed to realtime.
        auto delay = raisim::TimedLoop(m_base->get_simulator()->get_time_step() * 1e6);

        // Reset the wrench.
        wrench.setZero();

        const auto &state = m_base->get_frankaridgeback()->get_dynamics().get_end_effector_state();

        // Update the force component of the wrench.
        if (m_position) {
            Vector3d position = m_position->get_position(time);
            m_force_pid->set_reference(position);
            m_force_pid->update(state.position, time);
            m_force_pid_logger->log(*m_force_pid);

            // Update the visual sphere to show the tracked point.
            m_tracking_sphere->setPosition(position);

            wrench.head<3>() = m_force_pid->get_control();
        }

        // Update the torque component of the wrench.
        // if (m_orientation) {
        //     Quaterniond orientation = m_orientation->get_orientation(time);
        //     m_torque_pid->set_reference(orientation);
        //     m_torque_pid->update(state.orientation, time);
        //     m_torque_pid_logger->log(*m_torque_pid);
        //     wrench.tail<3>() = m_torque_pid->get_control();
        // }

        // Apply the wrench to the end effector.
        m_base->get_frankaridgeback()->add_end_effector_wrench(wrench, time);

        // if (m_base->get_frankaridgeback()->get_forecast()) {
        //     m_base->get_frankaridgeback()->get_forecast()->observe_wrench(wrench, time);
        // }

        // Step the base simulation.
        m_base->step();
    }

    return true;
}
