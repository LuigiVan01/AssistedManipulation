#pragma once

#include <filesystem>

#include "logging/csv.hpp"
#include "controller/mppi.hpp"
#include "frankaridgeback/objective/assisted_manipulation.hpp"

namespace logger {

/**
 * @brief Logging class for AssistedManipulation.
 */
class AssistedManipulation
{
public:

    struct Configuration {

        /// Path to the folder to log into.
        std::filesystem::path folder;

        /// Log the rollout costs.
        bool log_joint_cost = true;

        /// Log the rollout weights.
        bool log_reach_cost = true;

        /// Log the optimal control gradient.
        bool log_manipulability_cost = true;

        /// Log the optimal rollout.
        bool log_power_cost = true;

        /// Log the optimal rollout cost.
        bool log_variable_damping_cost = true;

        // JSON conversion for mppi logger configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, log_joint_cost, log_reach_cost, log_manipulability_cost,
            log_power_cost, log_variable_damping_cost
        )
    };

    /**
     * @brief Create a new AssistedManipulation logger.
     * 
     * @param configuration The logger configuration.
     * @returns A pointer to the AssistedManipulation logger on success or nullptr on failure.
     */
    static std::unique_ptr<AssistedManipulation> create(Configuration &&configuration);

    /**
     * @brief Log the objective function.
     * 
     * @param trajectory The trajectory that owns the objective.
     * @param objective The assisted manipulation objective.
     */
    void log(
        const mppi::Trajectory &trajectory,
        const ::AssistedManipulation &objective
    );

private:

    AssistedManipulation() = default;

    Configuration m_configuration;

    /// Calculation of the times of each horison step of the generator.
    std::vector<double> m_costs;

    /// Logger for 
    std::unique_ptr<CSV> m_logger;
};

} // namespace logger
