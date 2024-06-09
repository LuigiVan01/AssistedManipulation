#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "controller.hpp"

constexpr const std::size_t StateDoF = 10;
constexpr const std::size_t ControlDoF = 10;

/**
 * @brief The dynamics of the model parameters.
 * 
 * This class defines the dynamics of the franka research 3 attached to the
 * ridgeback mobile base.
 * 
 * A state constains:
 * - [x, y, yaw, joint1]
 * 
 * A control is defined by:
 * - [x, y, yaw, joint1]
 */
class Dynamics : public controller::Dynamics<StateDoF, ControlDoF>
{
public:
    
    static std::shared_ptr<Dynamics> create();

    /**
     * @brief Set the dynamics simulation to a given state.
     * 
     * @param state The system state.
     * @param t The time in the simulation.
     */
    inline void set(const State &state, double /* t */) override {
        m_state = state;
    };

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    State step(const Control &control, double dt) override;

private:

    /**
     * @brief Initialise the dynamics parameters.
     */
    Dynamics();

    /// The current dynamics parameters.
    State m_state;
};

/**
 * @brief The robot model.
 * 
 * This class can be updated with parameters and queried for the current robot
 * state.
 * 
 * This class uses pinnochio for calculating robot kinematics and dynamics.
 */
class Model
{
public:

    /// The state.
    using State = Eigen::Matrix<double, StateDoF, 1>;

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
