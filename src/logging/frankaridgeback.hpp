#include "logging/csv.hpp"

#include <filesystem>

#include "controller/json.hpp"
#include "frankaridgeback/dynamics.hpp"

namespace logger {

class FrankaRidgebackDynamics
{
public:

    struct Configuration {

        /// The folder to log the dynamics into.
        std::filesystem::path folder;

        /// Log the joint states.
        bool log_joints = true;

        /// Log the control input to the dynamics.
        bool log_control = true;

        /// Log the end effector position in world space.
        bool log_end_effector_position = true;

        /// Log the quaternion orientation in world space.
        bool log_end_effector_orientation = true;

        /// Log both the linear and angular end effector velocity.
        bool log_end_effector_velocity = true;

        /// Log both the linear and angular end effector acceleration.
        bool log_end_effector_acceleration = true;

        /// Log the power usage.
        bool log_power = true;

        /// Log the remaining tank energy.
        bool log_tank_energy = true;

        // JSON conversion for frankaridgeback dynamics logger configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            log_end_effector_position,
            log_end_effector_orientation,
            log_end_effector_velocity,
            log_end_effector_acceleration,
            log_power,
            log_tank_energy
        )
    };

    /**
     * @brief Create a logger for the frankaridgeback dynamics.
     * 
     * @param configuration The configuration of the frankaridgeback dynamics.
     * @returns The logger on success or nullptr on failure.
     */
    static std::unique_ptr<FrankaRidgebackDynamics> create(
        const Configuration &configuration
    );

    /**
     * @brief Log the state of the frankaridgeback.
     * 
     * @param time The time of the dynamics.
     * @param dynamics The dynamics to log.
     */
    void log(double time, const FrankaRidgeback::Dynamics &dynamics);

    /**
     * @brief Log a control.
     * 
     * @param time The time of the control.
     * @param control The control to log.
     */
    void log_control(double time, const VectorXd &control);

private:

    /// The configuration of the frankaridgeback dynamics logger.
    Configuration m_configuration;

    /// Optional logger for joint positions.
    std::unique_ptr<CSV> m_joint_logger;

    /// Optional logger for joint controls.
    std::unique_ptr<CSV> m_control_logger;

    /// Optional logger for end effector position.
    std::unique_ptr<CSV> m_position_logger;

    /// Optional logger for end effector orientation.
    std::unique_ptr<CSV> m_orientation_logger;

    /// Optional logger for end effector linear velocity.
    std::unique_ptr<CSV> m_linear_velocity_logger;

    /// Optional logger for end effector angular velocity.
    std::unique_ptr<CSV> m_angular_velocity_logger;

    /// Optional logger for end effector linear acceleration.
    std::unique_ptr<CSV> m_linear_acceleration_logger;

    /// Optional logger for end effector angular acceleration.
    std::unique_ptr<CSV> m_angular_acceleration_logger;

    /// Optional logger for power.
    std::unique_ptr<CSV> m_power_logger;

    /// Optional logger for energy.
    std::unique_ptr<CSV> m_energy_logger;
};

/**
 * @brief Logger for the frankaridgeback dynamics forecast.
 */
class FrankaRidgebackDynamicsForecast
{
public:

    struct Configuration {

        /// Folder to log into.
        std::filesystem::path folder = "";

        /// Log the joint states.
        bool log_joints = true;

        /// Log the end effector position in world space.
        bool log_end_effector_position = true;

        /// Log the quaternion orientation in world space.
        bool log_end_effector_orientation = true;

        /// Log both the linear and angular end effector velocity.
        bool log_end_effector_velocity = true;

        /// Log both the linear and angular end effector acceleration.
        bool log_end_effector_acceleration = true;

        /// Log the power usage.
        bool log_power = true;

        /// Log the remaining tank energy.
        bool log_tank_energy = true;

        /// Log the forecast wrench on the end effector.
        bool log_wrench = true;

        // JSON conversion for frankaridgeback dynamics forecast configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder,
            log_end_effector_position,
            log_end_effector_orientation,
            log_end_effector_velocity,
            log_end_effector_acceleration,
            log_power,
            log_tank_energy,
            log_wrench
        )
    };

    /**
     * @brief Create a logger for the frankaridgeback dynamics forecast.
     * 
     * @param configuration The configuration of the logger.
     * @returns The logger on success or nullptr on failure.
     */
    static std::unique_ptr<FrankaRidgebackDynamicsForecast> create(
        const Configuration &configuration
    );

    /**
     * @brief Log the forecast dynamics and rollout.
     * 
     * @param dynamics_forecast The forecast dynamics to log.
     */
    void log(const FrankaRidgeback::DynamicsForecast &dynamics_forecast);

private:

    Configuration m_configuration;

    /// Optional logger for joint positions.
    std::unique_ptr<CSV> m_joint_logger;

    /// The time of the last log.
    double last_forecast_time;

    /// Optional logger for end effector position.
    std::unique_ptr<CSV> m_position_logger;

    /// Optional logger for end effector orientation.
    std::unique_ptr<CSV> m_orientation_logger;

    /// Optional logger for end effector linear velocity.
    std::unique_ptr<CSV> m_linear_velocity_logger;

    /// Optional logger for end effector angular velocity.
    std::unique_ptr<CSV> m_angular_velocity_logger;

    /// Optional logger for end effector linear acceleration.
    std::unique_ptr<CSV> m_linear_acceleration_logger;

    /// Optional logger for end effector angular acceleration.
    std::unique_ptr<CSV> m_angular_acceleration_logger;

    /// Optional logger for power.
    std::unique_ptr<CSV> m_power_logger;

    /// Optional logger for energy.
    std::unique_ptr<CSV> m_energy_logger;

    /// Optional logger for forecast wrench.
    std::unique_ptr<CSV> m_wrench_logger;
};

} // namespace logger
