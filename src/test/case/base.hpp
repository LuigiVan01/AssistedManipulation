#pragma once

#include <cstdlib>
#include <filesystem>

#include "simulation/simulator.hpp"
#include "simulation/frankaridgeback.hpp"
#include "simulation/raisim_dynamics.hpp"
#include "frankaridgeback/objective/assisted_manipulation.hpp"
#include "frankaridgeback/objective/track_point.hpp"
#include "logging/mppi.hpp"
#include "logging/frankaridgeback.hpp"
#include "test/test.hpp"

class BaseTest : public RegisteredTest<BaseTest>
{
public:

    static inline constexpr const char *TEST_NAME = "base";

    /**
     * @brief Configuration of the objective function.
     */
    struct Objective {

        enum Type {
            ASSISTED_MANIPULATION,
            TRACK_POINT
        };

        /// The selected objective.
        Type type;

        /// Configuration for the assisted manipulation objective.
        std::optional<AssistedManipulation::Configuration> assisted_manipulation;

        /// Configuration for the reach objective.
        std::optional<TrackPoint::Configuration> track_point;

        // JSON conversion for base simulation objective.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Objective,
            type, assisted_manipulation, track_point
        )
    };

    struct Configuration {

        /// The output folder for the test.
        std::filesystem::path folder;

        /// Duration of the test.
        double duration;

        /// Simulation configuration.
        Simulator::Configuration simulator;

        /// The actors configuration including controller update rate.
        FrankaRidgeback::Actor::Configuration actor;

        /// The reach for point objective configuration.
        Objective objective;

        /// Optional force prediction for the dynamics.
        std::optional<Forecast::Configuration> wrench_forecast;

        /// MPPI logging configuration.
        logger::MPPI::Configuration mppi_logger;

        /// Franka-ridgeback logging configuration.
        logger::FrankaRidgebackDynamics::Configuration dynamics_logger;

        // JSON conversion for reach for point test configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, duration, simulator, actor, objective, wrench_forecast,
            mppi_logger, dynamics_logger
        )
    };

    static const Configuration DEFAULT_CONFIGURATION;

    /**
     * @brief Create a test reaching for a point.
     * 
     * @param options The test options. The configuration overrides from the
     * default configuration.
     * 
     * @return A pointer to the test on success or nullptr on failure.
     */
    static std::unique_ptr<BaseTest> create(Options &options);

    /**
     * @brief Create a the base simulation.
     * 
     * @param configuration The configuration of the base simulation.
     * @returns A pointer to the bast simulation on success or nullptr on failure.
     */
    static std::unique_ptr<BaseTest> create(const Configuration &configuration);

    /**
     * @brief Get the simulator.
     */
    inline Simulator *get_simulator()
    {
        return m_simulator.get();
    }

    /**
     * @brief Get the franka-ridgeback actor.
     */
    inline FrankaRidgeback::Actor *get_frankaridgeback()
    {
        return m_frankaridgeback.get();
    }

    /**
     * @brief Get the wrench forecast.
     * 
     * @note May be nullptr.
     */
    inline Forecast *get_wrench_forecast()
    {
        return m_wrench_forecast.get();
    }

    /**
     * @brief Step the simulation.
     * 
     * Can be called by other test cases to step the simulation, without having
     * to handle mppi logging.
     */
    void step();

    /**
     * @brief Run the test.
     * @returns If the test was successful.
     */
    bool run() override;

private:

    /**
     * @brief Initialise the base simulation.
     * 
     * @param configuration The configuration of the simulation.
     * @param simulator The simulator.
     * @param frankaridgeback The frankaridgeback instance being simulated.
     * @param mppi_logger Logger for the mppi.
     * @param dynamics_logger Logger for the simulated actor dynamics.
     */
    BaseTest(
        double duration,
        std::unique_ptr<Simulator> &&simulator,
        std::shared_ptr<FrankaRidgeback::Actor> &&frankaridgeback,
        std::unique_ptr<logger::MPPI> &&mppi_logger,
        std::unique_ptr<logger::FrankaRidgebackDynamics> &&dynamics_logger
    );

    /// Duration of the test when run.
    double m_duration;

    /// Pointer to the simulator.
    std::unique_ptr<Simulator> m_simulator;

    /// Pointer to the franka ridgeback being simulated.
    std::shared_ptr<FrankaRidgeback::Actor> m_frankaridgeback;

    /// Forecast to update wrench with.
    std::unique_ptr<Forecast> m_wrench_forecast;

    /// Logger for the frankaridgeback mppi trajectory generator.
    std::unique_ptr<logger::MPPI> m_mppi_logger;

    /// Logger for the frankaridgeback.
    std::unique_ptr<logger::FrankaRidgebackDynamics> m_dynamics_logger;
};
