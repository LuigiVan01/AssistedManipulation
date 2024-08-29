#pragma once

#include <cstdlib>
#include <filesystem>

#include "simulation/simulator.hpp"
#include "simulation/actors/frankaridgeback.hpp"
#include "frankaridgeback/dynamics.hpp"
#include "frankaridgeback/objective/track_point.hpp"
#include "logging/mppi.hpp"
#include "test/test.hpp"

class ReachForPoint : public RegisteredTest<ReachForPoint>
{
public:

    static inline constexpr const char *TEST_NAME = "reach";

    struct Configuration {

        /// The output folder for the test.
        std::filesystem::path folder;

        double duration;

        /// Simulation configuration.
        Simulator::Configuration simulator;

        /// Frankaridgeback dynamics configuration.
        FrankaRidgeback::Dynamics::Configuration dynamics;

        /// The reach for point objective configuration.
        TrackPoint::Configuration objective;

        /// The actors configuration including controller update rate.
        FrankaRidgebackActor::Configuration actor;

        /// MPPI logging configuration.
        logger::MPPI::Configuration logger;

        // JSON conversion for reach for point test configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, simulator, dynamics, objective, actor, logger
        )
    };

    static Configuration DEFAULT_CONFIGIURATION;

    /**
     * @brief Create a test reaching for a point.
     * 
     * @param options The test options. The configuration overrides from the
     * default configuration.
     * 
     * @return A pointer to the test on success or nullptr on failure.
     */
    static std::unique_ptr<Test> create(Options &options);

    static std::unique_ptr<Test> create(const Configuration &configuration);

    /**
     * @brief Run the test.
     * @returns If the test was successful.
     */
    bool run() override;

private:

    double m_duration;

    std::unique_ptr<Simulator> m_simulator;

    std::shared_ptr<FrankaRidgebackActor> m_robot;

    std::unique_ptr<logger::MPPI> m_mppi_logger;
};
