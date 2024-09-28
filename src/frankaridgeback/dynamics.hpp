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
    virtual Vector3d get_end_effector_position() const = 0;

    /**
     * @brief Get the orientation of the end effector in world space.
     */
    virtual Quaterniond get_end_effector_orientation() const = 0;

    /**
     * @brief Get the linear velocity (vx, vy, vz) of the end effector in world
     * space.
     */
    virtual Vector3d get_end_effector_linear_velocity() const = 0;

    /**
     * @brief Get the angular velocity (wx, wy, wz) of the end effector in world
     * space.
     */
    virtual Vector3d get_end_effector_angular_velocity() const = 0;

    /**
     * @brief Get the linear acceleration (ax, ay, az) of the end effector in
     * world space.
     */
    virtual Vector3d get_end_effector_linear_acceleration() const = 0;

    /**
     * @brief Get the angular acceleration (alpha_x, alpha_y, alpha_z) of the
     * end effector in world space.
     */
    virtual Vector3d get_end_effector_angular_acceleration() const = 0;

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
    virtual double get_power() const = 0;

    /**
     * @brief Get the current energy left in the energy tank.
     */
    virtual double get_tank_energy() const = 0;

    /**
     * @brief Get the end effector forecast wrench.
     * 
     * A prediction of the wrench applied to the end effector during rollout.
     * 
     * @returns The wrench (fx, fy, fz, tau_x, tau_y, tau_z) expected at the end
     * effector.
     */
    virtual Vector6d get_end_effector_forecast_wrench() const = 0;

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
    virtual Vector6d get_end_effector_virtual_wrench() const = 0;

    /**
     * @brief Get the actual wrench applied of the end effector by calls to
     * add_end_effector_true_wrench().
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @returns The actually applied wrench (fx, fy, fz, tau_x, tau_y, tau_z) at
     * the end effector in the world frame.
     */
    virtual Vector6d get_end_effector_simulated_wrench() const = 0;

    /**
     * @brief Add cumulative wrench to the end effector, to be simulated on the
     * next step, after which it is set to zero.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @param wrench The wrench to cumulative add to the end effector to be
     * simulated.
     */
    virtual void add_end_effector_simulated_wrench(Vector6d wrench) = 0;
};

} // namespace FrankaRidgeback
