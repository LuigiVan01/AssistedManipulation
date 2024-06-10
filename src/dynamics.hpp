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
        BASE = 0,
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

struct State : public Eigen::Matrix<double, DoF::STATE, 1>
{
    // Inherit all matrix constructors.
    using Eigen::Matrix<double, DoF::STATE, 1>::Matrix;

    /**
     * @brief Get the arm joint positions.
     * 
     * Slice of the first n = DoF::ARM elements.
     * 
     * @returns An eigen block of length FrankaRidgeback::DoF::ARM containing
     * the arm joint positions.
     */
    inline auto arm_position() {
        return head<DoF::ARM>();
    }

    /**
     * @brief Get the gripper position.
     * 
     * Slice of length <DoF::GRIPPER> starting at index (DoF::ARM)
     * 
     * @returns An eigen block of length FrankaRidgeback::DoF::GRIPPER
     * containing the gripper position.
     */
    inline auto gripper_position() {
        return segment<DoF::GRIPPER>(DoF::ARM);
    }

    /**
     * @brief Get all the joint positions of the robot.
     * 
     * Slice of the first n = DoF::JOINTS elements.
     * 
     * @returns An eigen block of length FrankaRidgeback::DoF::JOINTS containing
     * all joint positions.
     */
    inline auto joint_positions() {
        return head<DoF::JOINTS>();
    }

    /**
     * @brief Get the arm joint velocities.
     * 
     * Slice of length <DoF::ARM> starting at index (DoF::JOINTS)
     * 
     * @returns An eigen block of length FrankaRidgeback::DoF::ARM containing
     * the arm joint velocities.
     */
    inline auto arm_velocity() {
        return segment<DoF::ARM>(DoF::JOINTS);
    }

    /**
     * @brief Get the gripper joint velocities.
     * 
     * Slice of length <DoF::GRIPPER> starting at index (DoF::JOINTS + DoF::ARM)
     * 
     * @return An eigen block of length FrankaRidgeback::DoF::GRIPPER containing
     * the gripper joint velocities.
     */
    inline auto gripper_velocity() {
        return segment<DoF::GRIPPER>(DoF::JOINTS + DoF::ARM);
    }

    /**
     * @brief Get all the joint velocities of the robot.
     * 
     * Slice of length <DoF::JOINTS> starting at index (DoF::JOINTS)
     * 
     * @returns An eigen block of length FrankaRidgeback::DoF::JOINTS containing
     * all joint velocities.
     */
    inline auto joint_velocities() {
        return segment<DoF::JOINTS>(DoF::JOINTS);
    }

    /**
     * @brief Get the external torque.
     * 
     * Slice of the last n = FrankaRidgeback::DoF::EXTERNAL_TORQUE elements.
     * 
     * @returns An eigen block of length FrankaRidgeback::DoF::EXTERNAL_TORQUE
     * containing the external torque vector.
     */
    inline auto external_torque() {
        return tail<DoF::EXTERNAL_TORQUE>();
    }
};

struct Control : public Eigen::Matrix<double, DoF::CONTROL, 1>
{
    // Inherit all matrix constructors.
    using Eigen::Matrix<double, DoF::CONTROL, 1>::Matrix;

    /**
     * @brief Get the arm controller parameters.
     * 
     * Slice of the first n = DoF::ARM elements.
     * 
     * @returns The torques to apply to each sequential arm joint.
     */
    inline auto arm_torque() {
        return head<DoF::ARM>();
    }

    /**
     * @brief Get the gripper position control parameters.
     * 
     * Slice of the last n = DoF::GRIPPER elements.
     * 
     * @returns The desired position of the gripper.
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
