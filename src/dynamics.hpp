#pragma once

#include <utility>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "mppi.hpp"

namespace FrankaRidgeback {

// Various degrees of freedom of the franka research 3 ridgeback.
namespace DoF {

    // Base degrees of freedom.
    constexpr const std::size_t
        BASE_X = 1,
        BASE_Y = 1,
        BASE_YAW = 1,
        BASE_POSITION = BASE_X + BASE_Y,
        BASE = BASE_X + BASE_Y + BASE_YAW;

    // Arm degrees of freedom.
    constexpr const std::size_t
        ARM_JOINT1 = 1,
        ARM_JOINT2 = 1,
        ARM_JOINT3 = 1,
        ARM_JOINT4 = 1,
        ARM_JOINT5 = 1,
        ARM_JOINT6 = 1,
        ARM_JOINT7 = 1,
        ARM = ARM_JOINT1 + ARM_JOINT2 + ARM_JOINT3 + ARM_JOINT4 + ARM_JOINT5 + ARM_JOINT6 + ARM_JOINT7;

    // Gripper degrees of freedom.
    constexpr const std::size_t
        GRIPPER_LEFT = 1,
        GRIPPER_RIGHT = 1,
        GRIPPER = GRIPPER_LEFT + GRIPPER_RIGHT;

    // Joint dimensions.
    constexpr const std::size_t
        JOINTS = BASE + ARM + GRIPPER;

    // External torque.
    constexpr const std::size_t
        EXTERNAL_TORQUE = 1;

    /**
     * @brief Joint position, joint velocity and external torque.
     */
    constexpr const std::size_t STATE  = 2 * JOINTS + EXTERNAL_TORQUE;

    /**
     * @brief Base velocity, arm torque and gripper position.
     * 
     * See FrankaRidgeback::Control
     */
    constexpr const std::size_t CONTROL = JOINTS;

} // namespace DoF

/**
 * @brief The state of the robot.
 * 
 * @TODO: franka::RobotState?
 * 
 * The state is given by [x, y, yaw, theta1, theta2, theta3, theta4, theta5,
 * theta6, theta7, gripper_x, gripper_y, vx, vy, rotation/s, w1, w2, w3, w4, w5,
 * w6, w7, gripper_left_vx, gripper_right_vy, external_torque]
 * 
 * Units:
 * - Position in metres
 * - Velocities in metres per second.
 * - Torque in Newton metres.
 * - Angles in radians.
 * - Angular velocity in radians per second.
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
     * @brief Get the external torque.
     * 
     * Slice of the last n = <DoF::EXTERNAL_TORQUE> elements.
     * 
     * @returns The external torque applied to the end effector.
     */
    inline auto external_torque() {
        return tail<DoF::EXTERNAL_TORQUE>();
    }

    inline const auto external_torque() const {
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

/**
 * @brief The dynamics of the model parameters.
 * 
 * Defines how State evolves with Control input.
 */
class Dynamics : public mppi::Dynamics
{
public:

    static std::unique_ptr<Dynamics> create();

    inline constexpr int state_dof() override {
        return DoF::STATE;
    }

    inline constexpr int control_dof() override {
        return DoF::CONTROL;
    }

    /**
     * @brief Set the dynamics simulation to a given state.
     * 
     * @param state The system state.
     * @param t The time in the simulation.
     */
    inline void set(const Eigen::VectorXd &state) override {
        m_state = state;
    };

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) override;

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
     * @param end_effector_frame The name of the end effector frame.
     * 
     * @return A pointer to the model on success, or nullptr on failure.
     */
    static std::unique_ptr<Model> create(
        const std::string &filename,
        const std::string &end_effector_frame = "panda_grasp"
    );

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
