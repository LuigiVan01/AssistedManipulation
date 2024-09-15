#pragma once

#include <optional>

#include "frankaridgeback/dynamics.hpp"
#include "controller/mppi.hpp"
#include "controller/energy.hpp"
#include "controller/json.hpp"
#include "controller/forecast.hpp"
#include "simulation/simulator.hpp"

namespace FrankaRidgeback {

class RaisimDynamics : public Dynamics
{
public:

    struct Configuration {

        /// The simulator configuration.
        Simulator::Configuration simulator;

        bool enable_server;

        /// The file name of the robot definition.
        std::string filename;

        /// The frame of the end effector, to measure forces from.
        std::string end_effector_frame;

        /// The initial state.
        FrankaRidgeback::State initial_state;

        /// The proportional gain of the joint PD controller.
        FrankaRidgeback::Control proportional_gain;

        /// The differential gain of the joint PD controller.
        FrankaRidgeback::Control differential_gain;

        /// The initial available energy for the robot.
        double energy;

        // JSON conversion for franka-ridgeback raisim dynamics configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            simulator, filename, end_effector_frame, initial_state,
            proportional_gain, differential_gain
        )
    };

    static std::unique_ptr<RaisimDynamics> create(
        const Configuration &configuration,
        std::unique_ptr<Forecast::Handle> &&force_forecast
    );

    inline constexpr int control_dof() override {
        return FrankaRidgeback::DoF::CONTROL;
    }

    inline constexpr int state_dof() override {
        return FrankaRidgeback::DoF::STATE;
    }

    /**
     * @brief Get the previously set end effector force.
     */
    inline Eigen::Vector3d get_end_effector_force()
    {
        /// TODO: Check if this is correct. Why index zero? Documentation says
        // used for visualisation.
        return m_robot->getExternalForce()[0].e();
    }

    /**
     * @brief Get the previously set end effector torque.
     */
    inline Eigen::Vector3d get_end_effector_torque()
    {
        /// TODO: Check if this is correct. Why index zero? Documentation says
        // used for visualisation.
        return m_robot->getExternalTorque()[0].e();
    }

    /**
     * @brief Get the end effector position.
     * @returns The end effector position in world frame as (x, y, z).
     */
    inline Eigen::Vector3d get_end_effector_position() override
    {
        raisim::Vec<3> position;
        m_robot->getFramePosition(m_end_effector_frame_index, position);
        return position.e();
    }

    /**
     * @brief Get the end effector orientation.
     * @return The end effector orientation in world frame.
     */
    inline Eigen::Quaterniond get_end_effector_orientation() override
    {
        raisim::Mat<3, 3> orientation;
        m_robot->getFrameOrientation(m_end_effector_frame_index, orientation);
        return Eigen::Quaterniond(orientation.e());
    }

    /**
     * @brief Get the velocity of the end effector in the world frame.
     */
    inline Eigen::Vector3d get_end_effector_velocity() override {
        return m_end_effector_velocity;
    }

    /**
     * @brief Get the end effector jacobian, if calculation is enabled.
     * 
     * Jacobian is in the world frame.
     */
    inline Eigen::Ref<Eigen::Matrix<double, 6, DoF::JOINTS>> get_end_effector_jacobian() override {
        return m_end_effector_jacobian;
    }

    double get_power() override {
        return m_power;
    }

    /**
     * @brief Set the dynamics simulation to a given state.
     * @param state The system state.
     * @param time The time of the dynamincs state.
     */
    void set_state(const Eigen::VectorXd &state, double time) override;

    inline void set_end_effector_force(Eigen::Ref<Eigen::Vector3d> force) override {
        m_external_end_effector_force = force;
    }

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) override;

    /**
     * @brief Copy the dynamics.
     */
    inline std::unique_ptr<mppi::Dynamics> copy() override {
        if (m_forecast)
            return create(m_configuration, std::move(m_forecast->copy()));
        return create(m_configuration, nullptr);
    }

private:

    RaisimDynamics(
        const Configuration &configuration,
        std::unique_ptr<raisim::World> &&simulator,
        std::unique_ptr<Forecast::Handle> &&forecast_handle,
        raisim::ArticulatedSystem *robot,
        std::int64_t end_effector_frame_index
    );

    void update_kinematics();

    /// The configuration of the frankaridgeback dynamics.
    Configuration m_configuration;

    /// The simulated world.
    std::unique_ptr<raisim::World> m_world;

    /// The simulated articulated object, generated from the urdf file.
    raisim::ArticulatedSystem *m_robot;

    /// The current joint positions.
    Eigen::VectorXd m_position_command;

    /// The current joint velocities.
    Eigen::VectorXd m_velocity_command;

    /// The current joint positions.
    Eigen::VectorXd m_position;

    /// The current joint velocities.
    Eigen::VectorXd m_velocity;

    Eigen::VectorXd m_external_torque;

    std::unique_ptr<Forecast::Handle> m_forecast;

    /// The index of the end effector frame into raisim data structure.
    std::int64_t m_end_effector_frame_index;

    Eigen::MatrixXd m_end_effector_linear_jacobian;

    Eigen::MatrixXd m_end_effector_angular_jacobian;

    /// End effector jacobian in the world frame if enabled.
    Eigen::MatrixXd m_end_effector_jacobian;

    /// The velocity of the end effector if enabled.
    Eigen::Vector3d m_end_effector_velocity;

    Eigen::Vector3d m_external_end_effector_force;

    double m_power;

    /// The available energy.
    EnergyTank m_energy_tank;

    FrankaRidgeback::State m_state;

    double m_time;
};

} // FrankaRidgeback
