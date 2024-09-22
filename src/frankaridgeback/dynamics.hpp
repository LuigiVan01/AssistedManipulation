#pragma once

#include "controller/eigen.hpp"
#include "controller/mppi.hpp"
#include "frankaridgeback/dof.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

/**
 * @brief Matrix mapping of the end effector velocity to the joint velocities.
 */
using Jacobian = Eigen::Matrix<double, 6, DoF::JOINTS>;

/**
 * @brief Base class for all franka-ridgeback mppi dynamics implementations.
 * 
 * Used to ensure the derived dynamics class provides methods used by objective
 * functions.
 */
class Dynamics : public mppi::Dynamics
{
public:

    /**
     * @brief Static function that returns the path to the dynamics urdf file.
     * @returns Path to the robot definition.
     */
    static inline std::filesystem::path find_path() {
        return (std::filesystem::current_path() / "model/robot.urdf").string();
    }

    /**
     * @brief Get the position of the end effector in world space.
     */
    virtual Vector3d get_end_effector_position() = 0;

    /**
     * @brief Get the orientation of the end effector in world space.
     */
    virtual Quaterniond get_end_effector_orientation() = 0;

    /**
     * @brief Get the linear velocity (vx, vy, vz) of the end effector in world
     * space.
     */
    virtual Vector3d get_end_effector_linear_velocity() = 0;

    /**
     * @brief Get the angular velocity (wx, wy, wz) of the end effector in world
     * space.
     */
    virtual Vector3d get_end_effector_angular_velocity() = 0;

    /**
     * @brief Get the linear acceleration (ax, ay, az) of the end effector in
     * world space.
     */
    virtual Vector3d get_end_effector_linear_acceleration() = 0;

    /**
     * @brief Get the angular acceleration (alpha_x, alpha_y, alpha_z) of the
     * end effector in world space.
     */
    virtual Vector3d get_end_effector_angular_acceleration() = 0;

    /**
     * @brief Get the jacobian of the end effector.
     */
    virtual Eigen::Ref<Jacobian> get_end_effector_jacobian() = 0;

    /**
     * @brief Get the current dynamics power usage.
     * 
     * This is given by the sum of generalised joint force multiplied by their
     * generalised velocities. This is torque * angular velocity for revolute
     * joints and force * linear velocity for prismatic joints.
     * 
     * @return The current power usage in joules/s.
     */
    virtual double get_power() = 0;

    /**
     * @brief Get the end effector forecast wrench.
     * 
     * A prediction of the wrench applied to the end effector during rollout.
     * 
     * @returns The wrench (fx, fy, fz, tau_x, tau_y, tau_z) expected at the end
     * effector.
     */
    virtual Vector6d get_end_effector_forecast_wrench() = 0;

    /**
     * @brief Get the end effector virtual wrench.
     * 
     * The virtual wrench is the wrench that would be applied to the end
     * effector to produce its current linear and angular acceleration.
     * 
     * @note This can be compared against the forecasted end effector wrench to
     * reward rollouts that 
     * 
     * @returns A wrench (fx, fy, fz, tau_x, tau_y, tau_z) at the end effector
     * in the world frame, that results in the current end effector
     * acceleration.
     */
    virtual Vector6d get_end_effector_virtual_wrench() = 0;
};

} // namespace FrankaRidgeback
