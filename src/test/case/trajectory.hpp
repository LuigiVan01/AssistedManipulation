#pragma once

#include "test/test.hpp"
#include "simulation/simulator.hpp"
#include "controller/trajectory.hpp"

/**
 * @brief Test that the generated trajectories are sensical.
 */
class TrajectoryTest : public RegisteredTest<TrajectoryTest>
{
public:

    static inline constexpr const char *TEST_NAME = "trajectory";

    struct Configuration {

        /// The duration of the test.
        double duration;

        /// The position trajectory to visualise.
        std::optional<PositionTrajectory::Configuration> position;

        /// The orientation trajectory to visualise.
        std::optional<OrientationTrajectory::Configuration> orientation;

        // Durat
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            duration, position, orientation
        )
    };

    static const Configuration DEFAULT_CONFIGURATION;

    /**
     * @brief Create a new trajectory test.
     * @param options The options for the test.
     */
    static std::unique_ptr<TrajectoryTest> create(
        Options &options
    );

    /**
     * @brief Create a new trajectory test.
     * @param configuration The configuration of the test.
     */
    static std::unique_ptr<TrajectoryTest> create(
        const Configuration &configuration
    );

    /**
     * @brief Run the trajectory visualisation test.
     */
    bool run() override;

private:

    TrajectoryTest(
        const Configuration &configuration,
        std::unique_ptr<Simulator> &&simulator,
        std::unique_ptr<PositionTrajectory> &&position,
        std::unique_ptr<OrientationTrajectory> &&orientation,
        Vector3d origin
    );

    /// The duration of the test.
    double m_duration;

    /// Pointer to the simulator.
    std::unique_ptr<Simulator> m_simulator;

    /// The positional trajectory.
    std::unique_ptr<PositionTrajectory> m_position;

    /// The orientation trajectory.
    std::unique_ptr<OrientationTrajectory> m_orientation;

    /// Visual sphere at the centre of the position 
    raisim::Visuals *m_sphere;

    /// Points in the yaw/pitch direction of the orientation.
    raisim::Visuals *m_arrow;
};
