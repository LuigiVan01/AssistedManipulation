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

        bool log_joint_limit = true;

        bool log_minimise_velocity = true;

        bool log_self_collision = true;

        bool log_trajectory = true;

        bool log_reach = true;

        bool log_power = true;

        bool log_energy_tank = true;

        bool log_manipulability = true;

        bool log_variable_damping = true;

        // JSON conversion for mppi logger configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, log_joint_limit, log_minimise_velocity, log_self_collision,
            log_trajectory, log_reach, log_power, log_energy_tank,
            log_manipulability, log_variable_damping
        )
    };

    /**
     * @brief Create a new AssistedManipulation logger.
     * 
     * @param configuration The logger configuration.
     * @returns A pointer to the AssistedManipulation logger on success or nullptr on failure.
     */
    static std::unique_ptr<AssistedManipulation> create(Configuration configuration);

    /**
     * @brief Log the objective function.
     * 
     * @param time The time of the objective.
     * @param objective The assisted manipulation objective.
     */
    void log(
        double time,
        const FrankaRidgeback::AssistedManipulation &objective
    );

private:

    inline AssistedManipulation(const Configuration &configuration)
        : m_configuration(configuration)
    {}

    Configuration m_configuration;

    /// Calculation of the times of each horison step of the generator.
    std::vector<double> m_costs;

    /// Logger for 
    std::unique_ptr<CSV> m_logger;
};

} // namespace logger
