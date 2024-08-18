#pragma once

#include <Eigen/Eigen>

#include "controller/json.hpp"
#include "frankaridgeback/dof.hpp"

namespace FrankaRidgeback {

/**
 * @brief The data to send to the robot to control it.
 * 
 * The control is by [vx, vy, rotation, tau1, tau2, tau3, tau4, tau5, tau6, tau7,
 * gripper_x, gripper_y].
 * 
 * Units:
 * - Position in metres
 * - Velocities in metres per second.
 * - Torque in Newton metres.
 * - Angles in radians.
 * - Angular velocity in radians per second.
 * 
 * The base is omnidirectional, meaning vx and vy are independent from
 * each other, and both are relative to the rotation of the robot.
 */
struct Control : public Eigen::Vector<double, DoF::CONTROL>
{
    // Inherit all matrix constructors.
    using Eigen::Vector<double, DoF::CONTROL>::Vector;

    /**
     * @brief Get the vx and vy velocity [m/s] of the robot base.
     * 
     * Slice of the first n = <DoF::BASE> elements.
     * 
     * @return The (vx, vy) base velocity in metres per second.
     */
    inline auto base_velocity() {
        return head<DoF::BASE_POSITION>();
    }

    inline const auto base_velocity() const {
        return head<DoF::BASE_POSITION>();
    }

    /**
     * @brief Get the angle of rotation [rad/s] of the robot base.
     * 
     * Slice of length <DoF::BASE_YAW> starting at index
     * (Dof::BASE_VELOICTY).
     * 
     * @returns The angular velocity of the base in radians per second.
     */
    inline auto base_angular_velocity() {
        return segment<DoF::BASE_YAW>(DoF::BASE_POSITION);
    }

    inline const auto base_angular_velocity() const {
        return segment<DoF::BASE_YAW>(DoF::BASE_POSITION);
    }

    /**
     * @brief Get the torque [Nm] of the franka research 3 joints.
     * 
     * Slice of length <DoF::ARM> starting at index (DoF::BASE).
     * 
     * @returns The torques [tau1, tau2, tau3, tau4, tau5, tau6, tau7] ordered
     * from the base to the end effector, in newton metres.
     */
    inline auto arm_torque() {
        return segment<DoF::ARM>(DoF::BASE);
    }

    inline const auto arm_torque() const {
        return segment<DoF::ARM>(DoF::BASE);
    }

    /**
     * @brief Get the left and right gripper positions in metres from the middle.
     * 
     * Slice of the last n = <DoF::GRIPPER> elements.
     * 
     * @returns The left and right gripper positions in metres.
     */
    inline auto gripper_position() {
        return tail<DoF::GRIPPER>();
    }

    inline const auto gripper_position() const {
        return tail<DoF::GRIPPER>();
    }
};

inline void to_json(json& j, const Control &control) {
    to_json(j, (Eigen::VectorXd&)control);
}

inline void from_json(const json& j, Control &control) {
    from_json(j, (Eigen::VectorXd&)control);
}

} // namespace FrankaRidgeback
