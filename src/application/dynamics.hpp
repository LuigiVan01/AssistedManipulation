#pragma once

#include "controller/controller.hpp"

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>

constexpr const std::size_t StateDoF = 10;
constexpr const std::size_t ControlDoF = 10;

/**
 * @brief The model dynamics.
 * 
 * This class defines the dynamics of the franka research 3 attached to the
 * ridgeback mobile base.
 * 
 * A state constains:
 * - [x, y, yaw, joint1]
 * 
 * A control is defined by:
 * - [x, y, yaw, joint1]
 * 
 */
class FrankaResearch3RidgebackDyanamics : public controller::Dynamics<StateDoF, ControlDoF>
{
public:

    FrankaResearch3RidgebackDyanamics()
        : m_state(State::Zero())
    {}

    /**
     * @brief Set the dynamics simulation to a given state.
     * 
     * @param state The system state.
     * @param t The time in the simulation.
     */
    inline void set(const State &state, double t) override {
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

    State m_state;
};

class FrankaResearch3Ridgeback
{
    using State = Eigen::Matrix<double, StateDoF, 1>;

    FrankaResearch3Ridgeback() = default;

    /**
     * @brief Construct from an urdf definition.
     * 
     * @param urdf 
     * @throws A std::exception if the model fails to load.
     */
    FrankaResearch3Ridgeback(const std::string &urdf);

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
    void update(const State &state, const Velocity &velocity);

    /**
     * @brief Get the end effector pose.
     * @returns A tuple of (translation, rotation).
     */
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> end_effector();

private:

    /// The robot model.
    std::unique_ptr<pinocchio::Model> m_model;

    /// Data used for model calculations.
    std::unique_ptr<pinocchio::Data> m_data;

    /// Index of the end effector frame in the frame vector.
    int m_end_effector_index;
};