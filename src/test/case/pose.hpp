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

    static inline bool registration = []() {
        std::cout << "Pose test header included" << std::endl;
        return register_test();
    }();

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
        .position = PointTrajectory::DEFAULT_CONFIGURATION,
        .orientation = AxisAngleTrajectory::DEFAULT_CONFIGURATION
    };

    inline static std::unique_ptr<ExternalWrenchTest> create(
        Options &options
    ) {
        if (options.folder.empty())
            options.folder = "pose";

        /// @todo Currently the strategy of using a merge patch to change the
        /// test configuration does not work well with optional values being
        /// null values. A null in a merge patch means to explicitly delete
        /// that value from the json. Therefore, instead of setting something
        /// to std::nullopt, it actually removes the field entirely, resulting
        /// in a json parse error where different keys are not found.
        ///
        /// If a position or orientation overrides a std::nullopt, the fields
        /// for the remaining position and orientation are not present (they
        /// should actually be null), and this also results in a json parse
        /// error.

        json default_patch = {
            {"trajectory", {
                {"position", PositionTrajectory::Configuration()},
                {"orientation", OrientationTrajectory::Configuration()}
            }}
        };

        // Patch to set the reach object with the default configuration.
        default_patch.merge_patch(json::object({
            {"trajectory", {
                {"position", {
                    {"type", PositionTrajectory::Configuration::POINT},
                    {"point",  DEFAULT_CONFIGURATION.position}
                }},
                {"orientation", {
                    {"type", OrientationTrajectory::Configuration::AXIS_ANGLE},
                    {"axis_angle", DEFAULT_CONFIGURATION.orientation}
                }}
            }}
        }));

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
