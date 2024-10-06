#pragma once

#include "controller/trajectory.hpp"
#include "test/case/external_wrench.hpp"

/**
 * @brief Convenience test for maintaining a rectangluar trajectory with
 * external wrench.
 */
class RectangleTest : public RegisteredTest<RectangleTest>
{
public:

    static inline constexpr const char *TEST_NAME = "rectangle";

    struct Configuration {

        /// Configuration of the base external wrench test.
        ExternalWrenchTest::Configuration base;

        /// Patch to apply for the the rectangle trajectory.
        RectangularTrajectory::Configuration rectangle;

        // JSON conversion for rectangle test configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            base, rectangle
        )
    };

    inline const static Configuration DEFAULT_CONFIGURATION {
        .base = ExternalWrenchTest::DEFAULT_CONFIGURATION,
        .rectangle = RectangularTrajectory::DEFAULT_CONFIGURATION
    };

    inline static std::unique_ptr<ExternalWrenchTest> create(
        Options &options
    ) {
        if (options.folder.empty())
            options.folder = "rectangle";

        // Patch to set the reach object with the default configuration.
        json default_patch = {
            {"trajectory", {
                {"position", {
                    {"type", PositionTrajectory::Configuration::Type::RECTANGLE},
                    {"rectangle",  DEFAULT_CONFIGURATION.rectangle}
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
    RectangleTest() = default;
};
