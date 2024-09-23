#pragma once

#include <cstdlib>
#include <filesystem>

#include "test/case/base.hpp"
#include "simulation/actors/circle.hpp"
#include "logging/pid.hpp"

class Circle : public RegisteredTest<Circle>
{
public:

    static inline constexpr const char *TEST_NAME = "circle";

    struct Configuration {

        BaseSimulation::Configuration base;

        CircleActor::Configuration circle_actor;

        logger::MPPI::Configuration mppi_logger;

        logger::PID::Configuration pid_logger;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, simulator, objective, frankaridgeback_actor, circle_actor,
            mppi_logger, pid_logger
        )
    };

    static Configuration DEFAULT_CONFIGURATION;

    static std::unique_ptr<Test> create(const Configuration &configuration);

    /**
     * @brief Create a circle test instance.
     * 
     * @param options The test options. The configuration overrides from the
     * default configuration.
     * 
     * @returns A pointer to the test on success or nullptr on failure.
     */
    static std::unique_ptr<Test> create(Options &options);

    /**
     * @brief Run the test.
     * @returns If the test was successful.
     */
    bool run() override;

private:

    double m_duration;

    std::unique_ptr<Simulator> m_simulator;

    std::shared_ptr<FrankaRidgeback::Actor> m_robot;

    std::shared_ptr<CircleActor> m_actor;

    std::unique_ptr<logger::MPPI> m_mppi_logger;

    std::unique_ptr<logger::PID> m_pid_logger;
};
