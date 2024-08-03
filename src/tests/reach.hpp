#include <cstdlib>
#include <filesystem>

#include "simulation/simulator.hpp"
#include "simulation/actors/frankaridgeback.hpp"
#include "simulation/actors/circle.hpp"
#include "frankaridgeback/dynamics.hpp"
#include "frankaridgeback/objective/point.hpp"
#include "logging/mppi.hpp"

class ReachForPoint : public RegisteredTest
{
public:

    static const char *TEST_NAME = "reach";

    void run() override;

private:

    std::unique_ptr<Simulator> m_simulator;

    std::unique_ptr<FrankaRidgebackActor> m_robot;

    std::unique_ptr<logger::MPPI> m_mppi_logger;
};
