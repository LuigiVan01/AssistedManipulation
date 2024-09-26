#pragma once

#include "controller/trajectory.hpp"
#include "test/case/external_wrench.hpp"

/**
 * @brief Convenience test for maintaining pose with external wrench.
 */
class PoseTest : public RegisteredTest<PoseTest>
{
public:

    static inline constexpr const char *TEST_NAME = "pose";

    struct Configuration {

        /// Configuration of the base external wrench test.
        ExternalWrenchTest::Configuration base;

        /// Optional configuration of the positional trajectory.
        std::optional<PointTrajectory::Configuration> position;

        /// Optional configuration of the orientation trajectory.
        std::optional<AxisAngleTrajectory::Configuration> orientation;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            base, position, orientation
        )
    };

    inline const static Configuration DEFAULT_CONFIGURATION {
        .base = ExternalWrenchTest::DEFAULT_CONFIGURATION,
        .position = PointTrajectory::Configuration {
            .point = Vector3d(1.0, 1.0, 1.0)
        },
        .orientation = AxisAngleTrajectory::Configuration {
            .axis = Vector3d(0, 0, 1),
            .angle = 0.0
        }
    };

    inline static std::unique_ptr<ExternalWrenchTest> create(
        Options &options
    ) {
        if (options.folder.empty())
            options.folder = "pose";

        // Patch to set the reach object with the default configuration.
        json default_patch = {{
            {"trajectory", {
                "position", {
                    {"type", PositionTrajectory::Configuration::Type::POINT},
                    {"point",  DEFAULT_CONFIGURATION.position}
                },
                "orientation", {
                    {"type", OrientationTrajectory::Configuration::Type::AXIS_ANGLE},
                    {"orientation", DEFAULT_CONFIGURATION.orientation}
                }
            }}
        }};

        // Unless the options overrides the default itself.
        default_patch.merge_patch(options.patch);
        options.patch = default_patch;

        return ExternalWrenchTest::create(options);
    }

private:

    /**
     * @brief Disabled.
     */
    inline bool run() override
    {
        return false;
    }

    /// Disable construction.
    PoseTest() = default;
};
