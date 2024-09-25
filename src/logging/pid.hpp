#pragma once

#include <filesystem>

#include "logging/csv.hpp"
#include "controller/json.hpp"
#include "controller/pid.hpp"

namespace logger {

class PID
{
public:

    struct Configuration {

        /// Path to the log folder.
        std::filesystem::path folder;

        /// The number of degrees of freedom for the reference.
        unsigned int reference_dof;

        /// The number of degrees of freedom for the control.
        unsigned int control_dof;

        /// Log the reference.
        bool log_reference = true;

        /// Log the error.
        bool log_error = true;

        /// Log the cumulative error.
        bool log_cumulative_error = true;

        /// Log PID control saturation.
        bool log_saturation = true;

        /// Log PID controls.
        bool log_control = true;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, reference_dof, control_dof, log_reference, log_error,
            log_cumulative_error, log_saturation, log_control
        )
    };

    /**
     * @brief Create a new PID logger.
     * 
     * @param configuration The logger configuration.
     * @returns A pointer to the PID logger on success or nullptr on failure.
     */
    static std::unique_ptr<PID> create(const Configuration &configuration);

    /**
     * @brief Log a pid controller.
     * @param pid The pid controller to log.
     */
    void log(const controller::PID &pid);

private:

    PID() = default;

    /// Optional logger for mppi costs.
    std::unique_ptr<CSV> m_reference;

    /// Optional logger for mppi weights.
    std::unique_ptr<CSV> m_error;

    /// Optional logger for mppi gradient.
    std::unique_ptr<CSV> m_cumulative_error;

    /// Optional logger for the control saturation.
    std::unique_ptr<CSV> m_saturation;

    /// Optional logger for the control.
    std::unique_ptr<CSV> m_control;
};

} // namespace logger
