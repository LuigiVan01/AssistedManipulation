#pragma once

#include <optional>

#include "simulation/simulator.hpp"
#include "simulation/frankaridgeback/raisim_dynamics.hpp"
#include "controller/json.hpp"
#include "frankaridgeback/dynamics.hpp"
#include "frankaridgeback/pinocchio_dynamics.hpp"

namespace FrankaRidgeback {

/**
 * @brief Used to instantiate different simulated dynamics instances.
 */
class SimulatorDynamics
{
public:

    /**
     * @brief A common configuration for selecting different frankaridgeback
     * dynamics implementations.
     */
    struct Configuration {

        enum Type {
            RAISIM = 0,
            PINOCCHIO = 1
        };

        /// The type of configured adaptor.
        Type type;

        /// The configuration of the raisim dynamics adaptor if selected.
        std::optional<RaisimDynamics::Configuration> raisim;

        /// The configuration of the pinocchio dynamics adaptor if selected.
        std::optional<PinocchioDynamics::Configuration> pinocchio;

        // JSON conversion for simulator adaptor configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            type, raisim, pinocchio
        )
    };

    static std::unique_ptr<Dynamics> create(
        const Configuration &configuration,
        std::unique_ptr<DynamicsForecast::Handle> &&dynamics_forecast_handle = nullptr,
        Simulator *simulator = nullptr
    );

private:

    SimulatorDynamics() = delete;
};

} // namespace FrankaRidgeback
