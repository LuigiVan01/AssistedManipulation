#pragma once

#include "controller/trajectory.hpp"
#include "test/case/external_wrench.hpp"

/**
 * @brief Convenience test for a positional lissajous trajectory with wrench.
 */
class LissajousTest : public RegisteredTest<LissajousTest>
{
public:

    static inline constexpr const char *TEST_NAME = "lissajous";

    struct Configuration {

        /// Configuration of the test.
        ExternalWrenchTest::Configuration base;

        /// Patch to apply for the figure eight position trajectory.
        LissajousTrajectory::Configuration lissajous;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            base, lissajous
        )
    };

    inline const static Configuration DEFAULT_CONFIGURATION {
        .base = ExternalWrenchTest::DEFAULT_CONFIGURATION,
        .lissajous = {
            .origin = Vector3d(1.0, 1.0, 1.0),
            .x_amplitude = 1.0,
            .y_amplitude = 0.5,
            .z_amplitude = 0.0,
            .x_frequency = 0.5,
            .y_frequency = 1.5,
            .z_frequency = 0.0,
            .y_phase = M_PI / 2,
            .z_phase = 0.0
        }
    };

    inline static std::unique_ptr<ExternalWrenchTest> create(
        Options &options
    ) {
        if (options.folder.empty())
            options.folder = "lissajous";

        // Patch to set the reach object with the default configuration.
        json default_patch = {{
            {"trajectory", {
                "position", {
                    {"type", PositionTrajectory::Configuration::Type::LISSAJOUS},
                    {"lissajous",  DEFAULT_CONFIGURATION.lissajous}
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
    LissajousTest() = default;
};
