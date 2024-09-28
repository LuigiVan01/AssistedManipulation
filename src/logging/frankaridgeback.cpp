#include "logging/frankaridgeback.hpp"

namespace logger {

std::unique_ptr<FrankaRidgebackDynamics> FrankaRidgebackDynamics::create(
    const Configuration &configuration
) {
    auto logger = std::unique_ptr<FrankaRidgebackDynamics>(
        new FrankaRidgebackDynamics()
    );

    if (configuration.log_end_effector_position) {
        logger->m_position_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_position.csv",
            .header = CSV::make_header("time", "x", "y", "z")
        });
    }

    if (configuration.log_end_effector_orientation) {
        logger->m_orientation_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_orientation.csv",
            .header = CSV::make_header("time", "x", "y", "z", "w")
        });
    }

    if (configuration.log_end_effector_velocity) {
        logger->m_linear_velocity_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_linear_velocity.csv",
            .header = CSV::make_header("time", "x", "y", "z")
        });
        logger->m_angular_velocity_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_angular_velocity.csv",
            .header = CSV::make_header("time", "wx", "wy", "wz")
        });
    }

    if (configuration.log_end_effector_acceleration) {
        logger->m_linear_acceleration_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_linear_acceleration.csv",
            .header = CSV::make_header("time", "ax", "ay", "az")
        });
        logger->m_angular_acceleration_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_angular_acceleration.csv",
            .header = CSV::make_header("time", "alpha_x", "alpha_y", "alpha_z")
        });
    }

    if (configuration.log_power) {
        logger->m_power_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "power.csv",
            .header = CSV::make_header("time", "power")
        });
    }

    if (configuration.log_tank_energy) {
        logger->m_energy_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "tank_energy.csv",
            .header = CSV::make_header("time", "energy")
        });
    }

    bool error = (
        (configuration.log_end_effector_position && !logger->m_position_logger) ||
        (configuration.log_end_effector_orientation && !logger->m_orientation_logger) ||
        (configuration.log_end_effector_velocity && !logger->m_linear_velocity_logger) ||
        (configuration.log_end_effector_velocity && !logger->m_angular_velocity_logger) ||
        (configuration.log_end_effector_acceleration && !logger->m_linear_acceleration_logger) ||
        (configuration.log_end_effector_acceleration && !logger->m_angular_acceleration_logger) ||
        (configuration.log_power && !logger->m_power_logger) ||
        (configuration.log_tank_energy && !logger->m_energy_logger)
    );

    if (error) {
        std::cerr << "failed to create csv logger" << std::endl;
        return nullptr;
    }

    return logger;
}

void FrankaRidgebackDynamics::log(
    double time,
    const FrankaRidgeback::Dynamics &dynamics)
{
    if (m_position_logger) {
        m_position_logger->write(
            time,
            dynamics.get_end_effector_position()
        );
    }

    if (m_orientation_logger) {
        m_orientation_logger->write(
            time,
            dynamics.get_end_effector_orientation().coeffs()
        );
    }

    if (m_linear_velocity_logger) {
        m_linear_velocity_logger->write(
            time,
            dynamics.get_end_effector_linear_velocity()
        );
    }

    if (m_angular_velocity_logger) {
        m_angular_velocity_logger->write(
            time,
            dynamics.get_end_effector_angular_velocity()
        );
    }

    if (m_linear_acceleration_logger) {
        m_linear_acceleration_logger->write(
            time,
            dynamics.get_end_effector_linear_acceleration()
        );
    }

    if (m_angular_acceleration_logger) {
        m_angular_acceleration_logger->write(
            time,
            dynamics.get_end_effector_angular_acceleration()
        );
    }

    if (m_power_logger) {
        m_power_logger->write(time, dynamics.get_power());
    }

    if (m_energy_logger) {
        m_energy_logger->write(time, dynamics.get_tank_energy());
    }
}

} // namespace logger
