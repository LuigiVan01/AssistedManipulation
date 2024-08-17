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

    struct Configuration {

        /// The output folder for the test.
        std::string folder;

        Simulator::Configuration simulator;

        /// The reach for point objective configuration.
        TrackPoint::Configuration objective;

        /// Mppi configuration.
        mppi::Configuration mppi;

        /// The actors configuration including controller update rate.
        FrankaRidgebackActor::Configuration actor;

        logger::MPPI::Configuration mppi_logger;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, objective, mppi, actor
        );
    };

    static inline constexpr const char *TEST_NAME = "reach";

    /**
     * @brief Create a test reaching for a point.
     * 
     * @param configuration The configuration of the test for overriding, or
     * null.
     * 
     * @return std::unique_ptr<Test> 
     */
    static std::unique_ptr<Test> create(json &configuration);

    bool run() override;

private:

    std::unique_ptr<Simulator> m_simulator;

    std::shared_ptr<FrankaRidgebackActor> m_robot;

    std::unique_ptr<logger::MPPI> m_mppi_logger;
};
