#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "mppi.hpp"

namespace FrankaRidgeback {

// Various degrees of freedom of the franka research 3 ridgeback.
namespace DoF {

    // Joint dimensions.
    constexpr const std::size_t
        GRIPPER = 2,
        ARM = 7,
        BASE_VELOCITY = 2,
        BASE_ROTATION = 1,
        BASE = BASE_VELOCITY + BASE_ROTATION,
        JOINTS = GRIPPER + ARM + BASE;

    // External torque.
    constexpr const std::size_t
        EXTERNAL_TORQUE = 1;

    // The state contains all joint positions, all joint velocities, and
    // the external torque.
    constexpr const std::size_t STATE  = 2 * (GRIPPER + ARM + BASE) + EXTERNAL_TORQUE;

    // A control contains the desired joint torques of each joint.
    constexpr const std::size_t CONTROL = JOINTS;

} // namespace DoF

/**
 * @brief The state of the robot.
 * 
 * The state is given by [x, y, rotation, theta1, theta2, theta3, theta4,
 * theta5, theta6, theta7, gripper_x, gripper_y, vx, vy, rotation/s, w1, w2, w3,
 * w4, w5, w6, w7, gripper_left_x, gripper_right_y, external_torque]
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
        return head<DoF::BASE_VELOCITY>();
    }

    /**
     * @brief Get the angle of rotation [rad] of the robot base.
     * 
     * Slice of length <DoF::BASE_ROTATION> starting at index
     * (Dof::BASE_VELOICTY).
     * 
     * @returns The angle of rotation of the base in radians.
     */
    inline auto base_rotation() {
        return segment<DoF::BASE_ROTATION>(DoF::BASE_VELOCITY);
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

    /**
     * @brief Get the vx and vy velocity [m/s] of the robot base.
     * 
     * Slice of length <DoF::BASE_VELOCITY> starting at index (DoF::JOINTS).
     * 
     * @returns The (vx, vy) base velocity in metres per second.
     */
    inline auto base_velocity() {
        return segment<DoF::BASE_VELOCITY>(DoF::JOINTS);
    }

    /**
     * @brief Get the angular velocity [rad/s] of the robot base.
     * 
     * Slice of length <DoF::BASE_VELOCITY> starting at index
     * (DoF::JOINTS + DoF::BASE_VELOCITY).
     * 
     * @returns The change in angle of rotation of the base in radians per
     * second.
     */
    inline auto base_angular_velocity() {
        return segment<DoF::BASE_VELOCITY>(DoF::JOINTS + DoF::BASE_VELOCITY);
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

    /**
     * @brief Get the external torque.
     * 
     * Slice of the last n = <DoF::EXTERNAL_TORQUE> elements.
     * 
     * @returns The external torque applied to the end effector.
     */
    inline auto external_torque() {
        return tail<DoF::EXTERNAL_TORQUE>();
    }
};

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
        return head<DoF::BASE_VELOCITY>();
    }

    /**
     * @brief Get the angle of rotation [rad/s] of the robot base.
     * 
     * Slice of length <DoF::BASE_ROTATION> starting at index
     * (Dof::BASE_VELOICTY).
     * 
     * @returns The angular velocity of the base in radians per second.
     */
    inline auto base_angular_velocity() {
        return segment<DoF::BASE_ROTATION>(DoF::BASE_VELOCITY);
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
};

/**
 * @brief The dynamics of the model parameters.
 * 
 * Defines how State evolves with Control input.
 */
class Dynamics : public mppi::Dynamics<DoF::STATE, DoF::CONTROL>
{
public:

    static std::shared_ptr<Dynamics> create();

    /**
     * @brief Set the dynamics simulation to a given state.
     * 
     * @param state The system state.
     * @param t The time in the simulation.
     */
    inline void set(const State &state) override {
        m_state = state;
    };

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    const State &step(const Control &control, double dt) override;

private:

    /**
     * @brief Initialise the dynamics parameters.
     */
    Dynamics();

    /// The current state.
    State m_state;
};

/**
 * @brief The robot model.
 * 
 * This class can be updated with parameters and queried for the current robot
 * state.
 * 
 * This class uses pinocchio for calculating robot kinematics and dynamics.
 */
class Model
{
public:

    /**
     * @brief Create a new instance of the robot model.
     * 
     * @param filename The filename of the URDF robot definition file to
     * instantiate the model from.
     * 
     * @throws std::runtime_error If the model cannot be created.
     * 
     * @return A pointer to the model on success, or nullptr on failure.
     */
    static std::unique_ptr<Model> create(const std::string &filename);

    /**
     * @brief Update the state of the model.
     * 
     * @param state The joint parameters of the model.
     */
    void update(const State &state);

    /**
     * @brief Update the state of the model and the joint velocities.
     * 
     * @param state The joint parameters of the model.
     * @param velocity The velocities of each joint.
     */
    // void update(const State &state, const Velocity &velocity);

    /**
     * @brief Get the end effector pose.
     * @returns A tuple of (translation, rotation).
     */
    std::tuple<Eigen::Vector3d, Eigen::Quaterniond> end_effector();

private:

    Model() = default;

    /**
     * @brief Initialise the dynamics model.
     * 
     * To create an instance, use Model::create().
     * 
     * @param model Pointer to the pinnochio model.
     * @param data Pointer to the pinnochio data.
     * @param end_effector_index Index into the vector of joint frames for the
     * end effector.
     */
    Model(
        std::unique_ptr<pinocchio::Model> model,
        std::unique_ptr<pinocchio::Data> data,
        std::size_t end_effector_index
    );

    /// The robot model.
    std::unique_ptr<pinocchio::Model> m_model;

    /// Data used for model calculations.
    std::unique_ptr<pinocchio::Data> m_data;

    /// Index of the end effector frame in the frame vector.
    std::size_t m_end_effector_index;
};

} // namespace FrankaRidgeback
