#pragma once

#include "logging/log.hpp"
#include "controller/mppi.hpp"

class MPPILogger
{
public:

    struct Configuration {

        /// Optional logger for mppi costs.
        std::unique_ptr<CSVLogger> costs;

        /// Optional logger for mppi weights.
        std::unique_ptr<CSVLogger> weights;

        /// Optional logger for mppi gradient.
        std::unique_ptr<CSVLogger> gradient;

        /// Optional logger for the optimal rollout.
        std::unique_ptr<CSVLogger> optimal_rollout;

        /// Optional logger for the optimal rollout cost.
        std::unique_ptr<CSVLogger> optimal_cost;

        /// Optional logger for to calculation time of each update.
        std::unique_ptr<CSVLogger> iteration;
    };

    /**
     * @brief Create a new mppi logger.
     * 
     * @param configuration The optional loggers
     * @return std::unique_ptr<MPPILogger> 
     */
    static inline std::unique_ptr<MPPILogger> create(Configuration &&configuration) {
        return std::unique_ptr<MPPILogger>(
            new MPPILogger(std::move(configuration))
        );
    }

    /**
     * @brief Log the current trajectory.
     * @param trajectory The trajectory to log.
     */
    void log(const mppi::Trajectory &trajectory);

    const auto &get_time() const {
        return m_time;
    }

private:

    inline MPPILogger(Configuration &&configuration)
        : m_configuration(std::move(configuration))
        , m_time()
    {}

    Configuration m_configuration;

    /// Calculation of the times of each horison step of the generator.
    std::vector<double> m_time;
};
