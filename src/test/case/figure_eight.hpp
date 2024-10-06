#pragma once

#include "controller/trajectory.hpp"
#include "test/case/external_wrench.hpp"

/**
 * @brief Convenience test for maintaining a figure eight trajectory with
 * external wrench.
 */
class FigureEightTest : public RegisteredTest<FigureEightTest>
{
public:

    static inline constexpr const char *TEST_NAME = "figure_eight";

    struct Configuration {

        /// Configuration of the test.
        ExternalWrenchTest::Configuration base;

        /// Patch to apply for the figure eight position trajectory.
        FigureEightTrajectory::Configuration figure_eight;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            base, figure_eight
        )
    };

    inline const static Configuration DEFAULT_CONFIGURATION {
        .base = ExternalWrenchTest::DEFAULT_CONFIGURATION,
        .figure_eight = FigureEightTrajectory::DEFAULT_CONFIGURATION
    };

    inline static std::unique_ptr<ExternalWrenchTest> create(
        Options &options
    ) {
        if (options.folder.empty())
            options.folder = "figure_eight";

        // Patch to set the reach object with the default configuration.
        json default_patch = {
            {"trajectory", {
                {"position", {
                    {"type", PositionTrajectory::Configuration::Type::FIGURE_EIGHT},
                    {"figure_eight",  DEFAULT_CONFIGURATION.figure_eight}
                }}
            }}
        };

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
    FigureEightTest() = default;
};
