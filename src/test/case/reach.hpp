#pragma once

#include <cstdlib>
#include <filesystem>

#include "frankaridgeback/objective/track_point.hpp"
#include "test/case/base.hpp"

class ReachForPoint : public RegisteredTest<ReachForPoint>
{
public:

    static inline constexpr const char *TEST_NAME = "reach";

    static inline bool registration = []() {
        std::cout << "Reach test header included" << std::endl;
        return register_test();
    }();

    struct Configuration {

        /// The configuration of the base simulation.
        BaseTest::Configuration base;

        /// The reach for point objective configuration.
        FrankaRidgeback::TrackPoint::Configuration objective;

        // JSON conversion for reach for point test configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            base, objective
        )
    };

    inline const static Configuration DEFAULT_CONFIGIURATION {
        .base = BaseTest::DEFAULT_CONFIGURATION,
        .objective = FrankaRidgeback::TrackPoint::DEFAULT_CONFIGURATION
    };

    /**
     * @brief Create a test reaching for a point.
     * 
     * @param options The test options. The configuration overrides from the
     * default configuration.
     * 
     * @return A pointer to the test on success or nullptr on failure.
     */
    inline static std::unique_ptr<Test> create(Options &options)
    {
        if (options.folder.empty())
            options.folder = "reach";

        // Patch to set the reach object with the default configuration.
        json default_patch = {{
            "objective", {
                {"type", FrankaRidgeback::ObjectiveType::TRACK_POINT},
                {"track_point", DEFAULT_CONFIGIURATION.objective}
            }
        }};

        // Unless the options overrides the default itself.
        default_patch.merge_patch(options.patch);
        options.patch = default_patch;

        auto base = BaseTest::create(options);

        // Add a sphere to track the point being reached.
        if (base) {
            auto visual = base->get_simulator()->get_server().addVisualSphere(
                "tracking_sphere", 0.05
            );
            visual->setPosition(DEFAULT_CONFIGIURATION.objective.point);
        }

        return base;
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
    ReachForPoint() = default;
};
