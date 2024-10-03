#pragma once

#include "test/case/base.hpp"

/**
 * @brief Convenience test for testing the functionality of the pinocchio
 * dynamics for the robot and rollouts, using the assisted manipulation
 * objective function.
 * 
 * @warning The pinocchio dynamics is currently broken.
 */
class PinocchioDynamicsTest : public RegisteredTest<PinocchioDynamicsTest>
{
public:

    static inline constexpr const char *TEST_NAME = "pinocchio";

    struct Configuration {

        /// The duration of the test.
        double duration;

        /// The pinocchio dynamics configuration.
        FrankaRidgeback::PinocchioDynamics::Configuration dynamics;

        /// The initial state of the frankarigeback.
        FrankaRidgeback::State initial_state;

        // JSON conversion for pinocchio dynamics test configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            duration, dynamics, initial_state
        )
    };

    inline const static Configuration DEFAULT_CONFIGURATION {
        .duration = 300.0,
        .dynamics = FrankaRidgeback::PinocchioDynamics::DEFAULT_CONFIGURATION,
        .initial_state = FrankaRidgeback::State::Zero()
    };

    /**
     * @brief Create a instance of the pinocchio dynamics test
     * 
     * @param options The test options.
     * @return A pointer to the test on success or nullptr on failure.
     */
    inline static std::unique_ptr<Test> create(Options &options)
    {
        if (options.folder.empty())
            options.folder = "pinocchio";

        json default_patch = {{

        }};

        default_patch.merge_patch(options.patch);
        options.patch = default_patch;

        return BaseTest::create(options);
    }

private:

    /**
     * @brief Disabled
     */
    bool run() override;

    PinocchioDynamicsTest() = delete;
};
