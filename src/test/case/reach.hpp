#include <cstdlib>
#include <filesystem>

#include "simulation/simulator.hpp"
#include "simulation/actors/frankaridgeback.hpp"
#include "frankaridgeback/dynamics.hpp"
#include "frankaridgeback/objective/point.hpp"
#include "logging/mppi.hpp"
#include "test/test.hpp"

class ReachForPoint : public RegisteredTest<ReachForPoint>
{
public:

    static inline constexpr const char *TEST_NAME = "reach";

    static std::unique_ptr<Test> create();

    bool run() override;

private:

    std::unique_ptr<Simulator> m_simulator;

    std::shared_ptr<FrankaRidgebackActor> m_robot;

    std::unique_ptr<logger::MPPI> m_mppi_logger;
};
