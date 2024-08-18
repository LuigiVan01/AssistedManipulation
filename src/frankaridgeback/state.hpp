#pragma once

#include <Eigen/Eigen>

#include "controller/json.hpp"
#include "frankaridgeback/dof.hpp"

namespace FrankaRidgeback {

/**
 * @brief The state of the robot.
 * 
 * The state is given by (position, velocity, end effector forces) as
 * [
 *     x, y, yaw, theta1, theta2, theta3, theta4, theta5, theta6, theta7, gripper_x, gripper_y,
 *     vx, vy, rotation/s, w1, w2, w3, w4, w5, w6, w7, gripper_left_vx, gripper_right_vy,
 *     fx, fy, fz, tau_x, tau_y, tau_z
 * ]
 * 
 * Units:
 * - Position in metres
 * - Angles in radians.
 * - Velocities in metres per second.
 * - Angular velocity in radians per second.
 * - Force in Newtons.
 * - Torque in Newton metres.
 * 
 * The base is omnidirectional, meaning vx and vy are independent from
 * each other, and both are relative to the yaw of the robot.
 */
struct State : public Eigen::Vector<double, DoF::STATE>
{
    // Inherit all matrix constructors.
    using Eigen::Vector<double, DoF::STATE>::Vector;

    /**
     * @brief Get the (estimated) x and y position [m] of the robot base.
     * 
     * Slice of the first n = <DoF::BASE> elements.
     * 
     * @returns The estimated (x, y) base position in metres.
     */
    inline auto base_position() {
        return head<DoF::BASE_POSITION>();
    }

    inline const auto base_position() const {
        return head<DoF::BASE_POSITION>();
    }

    /**
     * @brief Get the angle of rotation [rad] of the robot base.
     * 
     * Slice of length <DoF::BASE_YAW> starting at index
     * (Dof::BASE_VELOICTY).
     * 
     * @returns The angle of rotation of the base in radians.
     */
    inline auto base_yaw() {
        return segment<DoF::BASE_YAW>(DoF::BASE_POSITION);
    }

    inline const auto base_yaw() const {
        return segment<DoF::BASE_YAW>(DoF::BASE_POSITION);
    }

    inline auto base() {
        return head<DoF::BASE>();
    }

    /**
     * @brief Get the angles [rad] of the franka research 3 joints.
     * 
     * Slice of length <DoF::ARM> starting at index (DoF::BASE).
     * 
     * @returns The angles [theta1, theta2, theta3, theta4, theta5, theta6,
     * theta7] ordered from the base to the end effector of each joint.
     */
    inline auto arm_position() {
        return segment<DoF::ARM>(DoF::BASE);
    }

    inline const auto arm_position() const {
        return segment<DoF::ARM>(DoF::BASE);
    }

    /**
     * @brief Get the left and right gripper positions in metres from the middle.
     * 
     * Slice of length <DoF::GRIPPER> starting at index (DoF::BASE + DoF::ARM).
     * 
     * @returns The (left, right) gripper positions in metres.
     */
    inline auto gripper_position() {
        return segment<DoF::GRIPPER>(DoF::BASE + DoF::ARM);
    }

    inline const auto gripper_position() const {
        return segment<DoF::GRIPPER>(DoF::BASE + DoF::ARM);
    }

    /**
     * @brief Get all the joint positions of the robot.
     * 
     * Slice of the first n = <DoF::JOINTS> elements.
     * 
     * The left gripper is x, and the right gripper is y.
     * 
     * @returns The joint positions [x, y, rotation, theta1, theta2, theta3,
     * theta4, theta5, theta6, theta7, gripper_x, gripper_y] of the
     * robot.
     */
    inline auto position() {
        return head<DoF::JOINTS>();
    }

    inline const auto position() const {
        return head<DoF::JOINTS>();
    }

    /**
     * @brief Get the vx and vy velocity [m/s] of the robot base.
     * 
     * Slice of length <DoF::BASE_POSITION> starting at index (DoF::JOINTS).
     * 
     * @returns The (vx, vy) base velocity in metres per second.
     */
    inline auto base_velocity() {
        return segment<DoF::BASE_POSITION>(DoF::JOINTS);
    }

    inline const auto base_velocity() const {
        return segment<DoF::BASE_POSITION>(DoF::JOINTS);
    }

    /**
     * @brief Get the angular velocity [rad/s] of the robot base.
     * 
     * Slice of length <DoF::BASE_POSITION> starting at index
     * (DoF::JOINTS + DoF::BASE_POSITION).
     * 
     * @returns The change in angle of rotation of the base in radians per
     * second.
     */
    inline auto base_angular_velocity() {
        return segment<DoF::BASE_POSITION>(DoF::JOINTS + DoF::BASE_POSITION);
    }

    inline const auto base_angular_velocity() const {
        return segment<DoF::BASE_POSITION>(DoF::JOINTS + DoF::BASE_POSITION);
    }

    /**
     * @brief Get the angular velocity [rad/s] of the franka research 3 joints.
     * 
     * Slice of length <DoF::ARM> starting at index (DoF::JOINTS + DoF::BASE).
     * 
     * @returns The angular velocity [w1, w2, w3, w4, w5, w6, w7] ordered from
     * the base to the end effector of each joint.
     */
    inline auto arm_velocity() {
        return segment<DoF::ARM>(DoF::JOINTS + DoF::BASE);
    }

    inline const auto arm_velocity() const {
        return segment<DoF::ARM>(DoF::JOINTS + DoF::BASE);
    }

    /**
     * @brief  Get the left and right gripper velocities [m/s].
     * 
     * Slice of length <DoF::GRIPPER> starting at index (DoF::JOINTS + DoF::ARM)
     * 
     * @return The (left, right) gripper positions in metres per second.
     */
    inline auto gripper_velocity() {
        return segment<DoF::GRIPPER>(DoF::JOINTS + DoF::BASE + DoF::ARM);
    }

    inline const auto gripper_velocity() const {
        return segment<DoF::GRIPPER>(DoF::JOINTS + DoF::BASE + DoF::ARM);
    }

    /**
     * @brief Get all the joint velocities of the robot.
     * 
     * Slice of length <DoF::JOINTS> starting at index (DoF::JOINTS)
     * 
     * The left gripper is x, and the right gripper is y.
     * 
     * @returns The joint velocities [vx, vy, rotation/s, w1, w2, w3, w4, w5,
     * w6, w7, gripper_left_x, gripper_right_y] of the robot.
     */
    inline auto velocity() {
        return segment<DoF::JOINTS>(DoF::JOINTS);
    }

    inline const auto velocity() const {
        return segment<DoF::JOINTS>(DoF::JOINTS);
    }

    /**
     * @brief Get the measured end effector torque.
     * 
     * Slice of the last <DoF::EXTERNAL_TORQUE> elements.
     * 
     * @returns The end effector torque [tau_x, tau_y, tau_z]
     */
    inline auto end_effector_torque() {
        return tail<DoF::EXTERNAL_TORQUE>();
    }

    inline const auto end_effector_torque() const {
        return tail<DoF::EXTERNAL_TORQUE>();
    }
};

inline void to_json(json& j, const State &state) {
    to_json(j, (Eigen::VectorXd&)state);
}

inline void from_json(const json& j, State &state) {
    from_json(j, (Eigen::VectorXd&)state);
}

} // namespace FrankaRidgeback
