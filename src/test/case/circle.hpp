#pragma once

#include <cstdlib>
#include <filesystem>

#include "simulation/simulator.hpp"
#include "simulation/actors/frankaridgeback.hpp"
#include "simulation/actors/circle.hpp"
#include "logging/mppi.hpp"
#include "logging/pid.hpp"
#include "test/test.hpp"

class Circle : public RegisteredTest<Circle>
{
public:

    static inline constexpr const char *TEST_NAME = "circle";

    static std::unique_ptr<Test> create();

    bool run() override;

private:

    std::unique_ptr<Simulator> m_simulator;

    std::shared_ptr<FrankaRidgebackActor> m_robot;

    std::shared_ptr<CircleActor> m_actor;

    std::unique_ptr<logger::MPPI> m_mppi_logger;

    std::unique_ptr<logger::PID> m_pid_logger;
};
