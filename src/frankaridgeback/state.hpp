#pragma once

#include "controller/eigen.hpp"
#include "controller/json.hpp"
#include "frankaridgeback/dof.hpp"

namespace FrankaRidgeback {

/**
 * @brief The state of the robot.
 * 
 * The state is given by (position, velocity, end effector forces) as
 * [
 *     x, y, yaw, theta1, theta2, theta3, theta4, theta5, theta6, theta7,
 *     gripper_x, gripper_y, vx, vy, rotation/s, w1, w2, w3, w4, w5, w6, w7,
 *     gripper_vx, gripper_vy, fx, fy, fz, tau_x, tau_y, tau_z, available_energy
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
    // inline auto gripper_position() {
    //     return segment<DoF::GRIPPER>(DoF::BASE + DoF::ARM);
    // }

    // inline const auto gripper_position() const {
    //     return segment<DoF::GRIPPER>(DoF::BASE + DoF::ARM);
    // }

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
     * @brief Get the measured end effector force.
     * 
     * Slice of length <DoF::EXTERNAL_FORCE> starting at index (2 * DoF::JOINTS).
     * 
     * @returns The end effector force [fx, fy, fz]
     */
    inline auto end_effector_force() {
        return segment<DoF::EXTERNAL_FORCE>(2 * DoF::JOINTS);
    }

    inline const auto end_effector_force() const {
        return segment<DoF::EXTERNAL_FORCE>(2 * DoF::JOINTS);
    }

    /**
     * @brief Get the measured end effector torque.
     * 
     * Slice of length <DoF::EXTERNAL_TORQUE> starting at index 2 * DoF::JOINTS + DoF::EXTERNAL_FORCE.
     * 
     * @returns The end effector torque [tau_x, tau_y, tau_z]
     */
    inline auto end_effector_torque() {
        return segment<DoF::EXTERNAL_TORQUE>(2 * DoF::JOINTS + DoF::EXTERNAL_FORCE);
    }

    inline const auto end_effector_torque() const {
        return segment<DoF::EXTERNAL_TORQUE>(2 * DoF::JOINTS + DoF::EXTERNAL_FORCE);
    }

    /**
     * @brief Get the measured end effector wrench.
     * 
     * Slice of length DoF::EXTERNAL_WRENCH starting at index 2 * DoF::JOINTS.
     * 
     * @returns The end effector wrench [fx, fy, fz, tau_x, tau_y, tau_z]
     */
    inline auto end_effector_wrench() {
        return segment<DoF::EXTERNAL_WRENCH>(2 * DoF::JOINTS);
    }

    inline const auto end_effector_wrench() const {
        return segment<DoF::EXTERNAL_WRENCH>(2 * DoF::JOINTS);
    }

    /**
     * @brief Get the available energy of the robot.
     * 
     * Slice of the last DoF::AVAILABLE_ENERGY elements.
     * 
     * @returns The available energy.
     */
    inline auto available_energy() {
        return tail<DoF::AVAILABLE_ENERGY>();
    }

    inline const auto available_energy() const {
        return tail<DoF::AVAILABLE_ENERGY>(); 
    }
};

/**
 * @brief ALl the selectable robot preset states.
 */
enum class Preset {
    ZERO = 0,
    HUDDLED_10J = 1
};

/**
 * @brief Get a preset state.
 * 
 * @param preset The preset state to get
 * @returns The state.
 */
State make_state(Preset preset);

/**
 * @brief Create an initial state.
 * 
 * @param position The position.
 * @param velocity The velocity.
 * @param wrench The wrench.
 * @param energy The energy.
 * 
 * @returns The resulting state.
 */
State make_state(
    VectorXd position,
    VectorXd velocity,
    Vector6d wrench,
    double energy
);

// JSON conversion should be handled by the generic type, but are custom here
// due to the issue in templated conversion function, see there.

inline void to_json(json& j, const State &state)
{
    j = json::array();
    for (int i = 0; i < state.size(); i++)
        j.push_back(state(i));
}

inline void from_json(const json& j, State &state)
{
    for (json::size_type i = 0; i < j.size(); i++)
        state(i) = j[i].get<double>();
}

} // namespace FrankaRidgeback
