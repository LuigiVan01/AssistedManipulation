#pragma once

#include "controller/trajectory.hpp"
#include "test/case/external_wrench.hpp"

/**
 * @brief Convenience test for oscillating and spherical interpolation between
 * two orientations with torque control via pid.
 */
class SlerpTrajectoryTest : public RegisteredTest<SlerpTrajectoryTest>
{
public:

    static inline constexpr const char *TEST_NAME = "slerp";

    struct Configuration {

        /// Configuration of the base external wrench test.
        ExternalWrenchTest::Configuration base;

        /// The position in space to maintain.
        PointTrajectory::Configuration point;

        /// Optional configuration of the positional trajectory.
        SlerpTrajectory::Configuration slerp;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            base, point, slerp
        )
    };

    inline const static Configuration DEFAULT_CONFIGURATION {
        .base = ExternalWrenchTest::DEFAULT_CONFIGURATION,
        .point = PointTrajectory::DEFAULT_CONFIGURATION,
        .slerp = SlerpTrajectory::DEFAULT_CONFIGURATION
    };

    inline static std::unique_ptr<ExternalWrenchTest> create(
        Options &options
    ) {
        if (options.folder.empty())
            options.folder = "slerp";

        // Patch to set the reach object with the default configuration.
        json default_patch = {{
            {"trajectory", {
                "position", {
                    {"type", PositionTrajectory::Configuration::Type::POINT},
                    {"point",  DEFAULT_CONFIGURATION.point}
                },
                "orientation", {
                    {"type", OrientationTrajectory::Configuration::Type::SLERP},
                    {"slerp", DEFAULT_CONFIGURATION.slerp}
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
    SlerpTrajectoryTest() = default;
};
