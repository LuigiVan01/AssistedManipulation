#pragma once

#include <filesystem>

#include "logging/csv.hpp"
#include "controller/mppi.hpp"

namespace logger {

/**
 * @brief Logging class for MPPI.
 */
class MPPI
{
public:

    struct Configuration {

        /// Path to the folder to log into.
        std::filesystem::path folder;

        /// The trajectory to log, for logger initialisation.
        const mppi::Trajectory *trajectory;
    };

    /**
     * @brief Create a new MPPI logger.
     * 
     * @param configuration The logger configuration.
     * @returns A pointer to the MPPI logger on success or nullptr on failure.
     */
    static std::unique_ptr<MPPI> create(Configuration &&configuration);

    /**
     * @brief Log the current trajectory.
     * @param trajectory The trajectory to log.
     */
    void log(const mppi::Trajectory &trajectory);

private:

    MPPI(
        std::unique_ptr<CSV> &&costs,
        std::unique_ptr<CSV> &&weights,
        std::unique_ptr<CSV> &&gradient,
        std::unique_ptr<CSV> &&optimal_rollout,
        std::unique_ptr<CSV> &&optimal_cost,
        std::unique_ptr<CSV> &&iteration
    );

    /// The last time the trajectory was updated.
    double m_last_update;

    /// Calculation of the times of each horison step of the generator.
    std::vector<double> m_time;

    /// Optional logger for mppi costs.
    std::unique_ptr<CSV> m_costs;

    /// Optional logger for mppi weights.
    std::unique_ptr<CSV> m_weights;

    /// Optional logger for mppi gradient.
    std::unique_ptr<CSV> m_gradient;

    /// Optional logger for the optimal rollout.
    std::unique_ptr<CSV> m_optimal_rollout;

    /// Optional logger for the optimal rollout cost.
    std::unique_ptr<CSV> m_optimal_cost;

    /// Optional logger for to calculation time of each update.
    std::unique_ptr<CSV> m_update;
};

} // namespace logger
