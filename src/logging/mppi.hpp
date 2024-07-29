#pragma once

#include "logging/log.hpp"
#include "controller/mppi.hpp"

class MPPILogger
{
public:

    struct Configuration {

        std::string folder;

        /// Log file name for mppi weights.
        std::optional<std::string> weights;

        /// Log file name for mppi gradient.
        std::optional<std::string> gradient;

        /// Log file name for the optimal rollout.
        std::optional<std::string> optimal_rollout;

        /// Log file name for the optimal rollout cost.
        std::optional<std::string> optimal_cost;

        /// Log file name for to calculation time of each update.
        std::optional<std::string> update_time;
    };

    inline std::unique_ptr<MPPILogger> create(Configuration &&configuration)
    {
        return std::unique_ptr<MPPILogger>(
            new MPPILogger
        );
    }

    /**
     * @brief Log the current trajectory.
     * @param trajectory The trajectory to log.
     */
    void log(const mppi::Trajectory &trajectory);

private:

    void log_weights(const mppi::Trajectory &trajectory);

    /// Logger for mppi weights.
    std::unique_ptr<CSVLogger> m_weights;

    /// Logger for mppi gradient.
    std::unique_ptr<CSVLogger> m_gradient;

    /// Logger for mppi optimal rollout.
    std::unique_ptr<CSVLogger> m_optimal_rollout;

    /// Logger for mppi optimal costs.
    std::unique_ptr<CSVLogger> m_optimal_cost;

    /// Logger for mppi update time.
    std::unique_ptr<CSVLogger> m_update_time;
};
