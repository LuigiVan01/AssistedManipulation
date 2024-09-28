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
     * Note that the dynamics should really be const, but returning const 
     * Eigen::Ref<Jacobian> is complaining.
     * 
     * @param time The time of the dynamics.
     * @param dynamics The dynamics to log.
     */
    void log(double time, const FrankaRidgeback::Dynamics &dynamics);

private:

    /// The configuration of the frankaridgeback dynamics logger.
    Configuration m_configuration;

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

} // namespace logger
