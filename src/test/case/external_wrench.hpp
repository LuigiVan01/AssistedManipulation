#pragma once

#include <cstdlib>
#include <filesystem>

#include "test/case/base.hpp"
#include "controller/trajectory.hpp"
#include "controller/pid.hpp"
#include "logging/pid.hpp"

class ExternalWrenchTest : public RegisteredTest<ExternalWrenchTest>
{
public:

    static inline constexpr const char *TEST_NAME = "external_wrench";

    struct Configuration {

        /// Folder to save simulation data to.
        std::filesystem::path folder;

        /// Duration of the simulation.
        double duration;

        /// Configuration of the base mppi frankaridgeback simulation.
        BaseTest::Configuration base;

        struct Trajectory {

            /// The positional component of the external trajectory.
            std::optional<PositionTrajectory::Configuration> position;

            /// The orientation component of the external trajectory.
            std::optional<OrientationTrajectory::Configuration> orientation;

            /// JSON conversion for external wrench trajectory configuration.
            NLOHMANN_DEFINE_TYPE_INTRUSIVE(
                Trajectory,
                position, orientation
            )
        };

        /// The desired trajectory of the external force.
        Trajectory trajectory;

        /// The pid controller configuration for handling applied force at the
        /// end effector.
        controller::PID::Configuration force_pid;

        /// The pid controller configuration for handling the applied torque
        /// at the end effector.
        controller::QuaternionPID::Configuration torque_pid;

        /// JSON conversion for external wrench simulation configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, duration, base, trajectory, force_pid, torque_pid
        )
    };

    static inline Configuration DEFAULT_CONFIGURATION = {
        .folder = "external_wrench",
        .duration = 15,
        .base = BaseTest::DEFAULT_CONFIGURATION,
        .trajectory = {
            // .position = std::nullopt,
            .position = PositionTrajectory::Configuration {
                .type = PositionTrajectory::Configuration::CIRCLE,
                .point = PointTrajectory::DEFAULT_CONFIGURATION,
                .circle = CircularTrajectory::DEFAULT_CONFIGURATION,
                .rectangle = RectangularTrajectory::DEFAULT_CONFIGURATION,
                .lissajous = LissajousTrajectory::DEFAULT_CONFIGURATION,
                .figure_eight = FigureEightTrajectory::DEFAULT_CONFIGURATION
            },
            .orientation = std::nullopt
            // .orientation = OrientationTrajectory::Configuration {
            //     .type = OrientationTrajectory::Configuration::AXIS_ANGLE,
            //     .axis_angle = std::nullopt,
            //     .slerp = std::nullopt
            // }
        },
        .force_pid = controller::PID::HUMAN_POINT_CONTROL,
        .torque_pid = controller::QuaternionPID::HUMAN_ORIENTATION_CONTROL
    };

    /**
     * @brief Create an external wrench simulation.
     * 
     * @param configuration The configuration of the external wrench simulation.
     * @return A pointer to the simulation on success or nullptr on failure.
     */
    static std::unique_ptr<ExternalWrenchTest> create(Configuration configuration);

    /**
     * @brief Create a circle test instance.
     * 
     * @param options The test options. The configuration overrides from the
     * default configuration.
     * 
     * @returns A pointer to the test on success or nullptr on failure.
     */
    static std::unique_ptr<ExternalWrenchTest> create(Options &options);

    /**
     * @brief Run the test.
     * @returns If the test was successful.
     */
    bool run() override;

private:

    /**
     * @brief Initialise the external wrench simulation.
     */
    ExternalWrenchTest(
        Configuration &&configuration,
        std::unique_ptr<BaseTest> &&base,
        std::unique_ptr<PositionTrajectory> &&position,
        std::unique_ptr<OrientationTrajectory> &&orientation,
        std::unique_ptr<controller::PID> &&force_pid,
        std::unique_ptr<controller::QuaternionPID> &&torque_pid,
        std::unique_ptr<logger::PID> &&force_pid_logger,
        std::unique_ptr<logger::PID> &&torque_pid_logger
    );

    /// The configuration of the external wrench simulation.
    Configuration m_configuration;

    /// Pointer to the base simulation, interacted with to implement the
    /// simulation.
    std::unique_ptr<BaseTest> m_base;

    /// A positional trajectory for the force pid controller to follow.
    std::unique_ptr<PositionTrajectory> m_position;

    /// An orientation trajectory for the torque pid controller to follow.
    std::unique_ptr<OrientationTrajectory> m_orientation;

    /// The optional pid controller applying external force to the end effector.
    std::unique_ptr<controller::PID> m_force_pid;

    /// The optional torque controller applying external torque to the end
    /// effector.
    std::unique_ptr<controller::QuaternionPID> m_torque_pid;

    /// The pid controller mapping the position trajectory to applied end
    /// effector force.
    std::unique_ptr<logger::PID> m_force_pid_logger;

    /// The pid controller mapping the orientation trajectory to applied end
    /// effector torque.
    std::unique_ptr<logger::PID> m_torque_pid_logger;

    /// Visual sphere of the point being tracked.
    raisim::Visuals *m_tracking_sphere;
};
