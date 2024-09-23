#pragma once

#include <optional>

#include "frankaridgeback/dynamics.hpp"
#include "controller/mppi.hpp"
#include "controller/energy.hpp"
#include "controller/json.hpp"
#include "controller/forecast.hpp"
#include "simulation/simulator.hpp"

namespace FrankaRidgeback {

/**
 * @brief MPPI frankaridgeback dynamics implemented with RaiSim.
 * 
 * This class is responsible for both:
 * - Simulating the FrankaRidgeback robot in the main simulation. The resulting
 *   state is passed to the FrankaRidgeback::Actor and visualised.
 * - Performing forward simulated rollouts of the dynamics in the MPPI routine.
 */
class RaisimDynamics : public Dynamics
{
public:

    struct Configuration {

        /// The simulator configuration if the simulator is not passed
        std::optional<Simulator::Configuration> simulator;

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

    /**
     * @brief Create a new franka-ridgeback raisim dynamics instance.
     * 
     * Copies the configuration.
     * 
     * @param configuration The configuration of the raisim dynamics.
     * @param force_forecast Handle to the force forecaster.
     * @param world Optional pointer to the world in which to simulate the
     * dynamics. If nullptr then the simulator
     * 
     * @returns A pointer to the dynamics on success or nullptr on failure.
     */
    static std::unique_ptr<RaisimDynamics> create(
        Configuration configuration,
        std::unique_ptr<Forecast::Handle> &&force_forecast,
        std::shared_ptr<raisim::World> world = nullptr
    );

    /**
     * @brief Copy the dynamics.
     * @returns A copy of the dynamics.
     */
    inline std::unique_ptr<mppi::Dynamics> copy() override {
        if (m_forecast)
            return create(m_configuration, std::move(m_forecast->copy()));
        return create(m_configuration, nullptr);
    }

    /**
     * @brief Step the dynamics.
     * 
     * Called when used for mppi trajectory generation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     * 
     * @returns The subsequent state.
     */
    Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) override;

    /**
     * @brief Set the control actions for the dynamics.
     * 
     * Called when used for simulating in an external world.
     * 
     * @param control The controls applied at the current state (before dt).
     */
    void act(const Control &control);

    /**
     * @brief Update the state of the dynamics.
     * 
     * Called when used for simulating in an external world.
     */
    void update();

    /**
     * @brief Set the state of the dynamics.
     * 
     * The wrench of the state is ignored, and is instead handled by the force
     * forecast.
     * 
     * @param state The system state.
     * @param time The time of the state.
     */
    void set_state(const Eigen::VectorXd &state, double time) override;

    /**
     * @brief Get the dynamics state.
     * @returns The state of the dynamics.
     */
    Eigen::Ref<Eigen::VectorXd> get_state() override
    {
        return m_state;
    }

    /**
     * @brief Get the degrees of freedom of the franka-ridgeback control.
     */
    inline constexpr int get_control_dof() override
    {
        return FrankaRidgeback::DoF::CONTROL;
    }

    /**
     * @brief Get the degrees of freedom of the franka-ridgeback state.
     */
    inline constexpr int get_state_dof() override
    {
        return FrankaRidgeback::DoF::STATE;
    }

    /**
     * @brief Get the position of the end effector in world space.
     */
    inline Vector3d get_end_effector_position() override
    {
        raisim::Vec<3> position;
        m_robot->getFramePosition(m_end_effector_frame_index, position);
        return position.e();
    }

    /**
     * @brief Get the orientation of the end effector in world space.
     */
    inline Quaterniond get_end_effector_orientation() override
    {
        raisim::Mat<3, 3> orientation;
        m_robot->getFrameOrientation(m_end_effector_frame_index, orientation);
        return Eigen::Quaterniond(orientation.e());
    }

    /**
     * @brief Get the linear velocity (vx, vy, vz) of the end effector in world
     * space.
     */
    inline Vector3d get_end_effector_linear_velocity() override
    {
        return m_end_effector_linear_velocity;
    }

    /**
     * @brief Get the angular velocity (wx, wy, wz) of the end effector in world
     * space.
     */
    inline Vector3d get_end_effector_angular_velocity() override
    {
        return m_end_effector_angular_velocity;
    }

    /**
     * @brief Get the linear acceleration (ax, ay, az) of the end effector in
     * world space.
     */
    inline Vector3d get_end_effector_linear_acceleration() override
    {
        return m_end_effector_linear_acceleration;
    }

    /**
     * @brief Get the angular acceleration (alpha_x, alpha_y, alpha_z) of the
     * end effector in world space.
     */
    inline Vector3d get_end_effector_angular_acceleration() override
    {
        return m_end_effector_angular_acceleration;
    }

    /**
     * @brief Get the jacobian of the end effector.
     */
    Eigen::Ref<Jacobian> get_end_effector_jacobian() override
    {
        return m_end_effector_jacobian;
    }

    /**
     * @brief Get the current dynamics power usage.
     * 
     * This is given by the sum of generalised joint force multiplied by their
     * generalised velocities. This is torque * angular velocity for revolute
     * joints and force * linear velocity for prismatic joints.
     * 
     * @return The current power usage in joules/s.
     */
    inline double get_power() override
    {
        return m_power;
    }

    /**
     * @brief Get the end effector forecast wrench.
     * 
     * A prediction of the wrench applied to the end effector during rollout.
     * 
     * @returns The wrench (fx, fy, fz, tau_x, tau_y, tau_z) expected at the end
     * effector.
     */
    inline Vector6d get_end_effector_forecast_wrench() override
    {
        return m_end_effector_forecast_wrench;
    }

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
    inline Vector6d get_end_effector_virtual_wrench() override
    {
        return m_end_effector_virtual_wrench;
    }

    /**
     * @brief Get the actual wrench of the end effector.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @returns The actually applied wrench (fx, fy, fz, tau_x, tau_y, tau_z) at
     * the end effector in the world frame.
     */
    inline Vector6d get_end_effector_true_wrench()
    {
        return m_end_effector_virtual_wrench;
    }

    /**
     * @brief Add cumulative wrench to the end effector, to be simulated on the
     * next step.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * After the next step, the end effector true wrench is set to zero.
     * 
     * @param wrench The wrench to cumulative add to the end effector to be
     * simulated.
     */
    void add_end_effector_true_wrench(Eigen::Ref<Vector6d> wrench);

    /**
     * @brief Get the previously set end effector force.
     */
    inline Vector3d get_end_effector_true_force()
    {
        /// TODO: Check if this is correct. Why index zero? Documentation says
        // used for visualisation.
        return m_robot->getExternalForce()[0].e();
    }

    /**
     * @brief Get the previously set end effector torque.
     */
    inline Vector3d get_end_effector_true_torque()
    {
        /// TODO: Check if this is correct. Why index zero? Documentation says
        // used for visualisation.
        return m_robot->getExternalTorque()[0].e();
    }

private:

    /**
     * @brief Initialise the raisim franka-ridgeback dynamics.
     * 
     * @param configuration 
     * @param world The world in which the franka-ridgeback exists.
     * @param forecast_handle Handle to the external wrench forecast.
     * @param robot Pointer to the robot in the world.
     * @param end_effector_frame_index Index of the end effector frame.
     * @param end_effector_frame_name The name of the end effector frame.
     */
    RaisimDynamics(
        const Configuration &configuration,
        std::shared_ptr<raisim::World> &&world,
        std::unique_ptr<Forecast::Handle> &&forecast_handle,
        raisim::ArticulatedSystem *robot,
        std::int64_t end_effector_frame_index,
        std::string end_effector_frame_name
    );

    /**
     * @brief Updates derived kinematic information.
     * 
     * Called on setting the state and every step.
     * 
     * Responsible for:
     * - Getting the end effector jacobian.
     * - Calculating the power consumption.
     * - Calculating end effector velocity.
     * - Calculating end effector acceleration.
     */
    void calculate();

    /**
     * @brief Calculates the virtual end effector wrench.
     * 
     * The virtual wrench is the wrench that would be applied to the end
     * effector to produce its current linear and angular acceleration.
     */
    Vector6d calculate_virtual_end_effector_wrench();

    /// The configuration of the frankaridgeback dynamics.
    Configuration m_configuration;

    /// The simulated world.
    std::shared_ptr<raisim::World> m_world;

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

    /// Handle to the forecast to get forecast wrench from.
    std::unique_ptr<Forecast::Handle> m_forecast;

    /// The index of the end effector frame into raisim data structure.
    std::int64_t m_end_effector_frame_index;

    /// The name of the end effector frame of reference.
    std::string m_end_effector_frame_name;

    /// Linear part of the jacobian, returned by raisim.
    Eigen::MatrixXd m_end_effector_linear_jacobian;

    /// Angular part of the jacobian, returned by raisim.
    Eigen::MatrixXd m_end_effector_angular_jacobian;

    /// Combined linear and angular end effector jacobian in the world frame.
    Jacobian m_end_effector_jacobian;

    /// The linear velocity (vx, vy, vz) of the end effector in the world frame.
    Vector3d m_end_effector_linear_velocity;

    /// The angular velocity (wx, wy, wz) of the end effector in the world frame. 
    Vector3d m_end_effector_angular_velocity;

    /// The linear acceleration of the end effector in the world frame.
    Vector3d m_end_effector_linear_acceleration;

    /// The angular acceleration of the end effector in the world frame.
    Vector3d m_end_effector_angular_acceleration;

    /// A prediction of the wrench applied to the end effector during rollout.
    Vector6d m_end_effector_forecast_wrench;

    /// The virtual wrench is the wrench that would be applied to the end
    /// effector to produce its current linear and angular acceleration.
    Vector6d m_end_effector_virtual_wrench;

    /// Cumulative wrench added to the end effector, only for simulation.
    Vector6d m_end_effector_true_wrench;

    /// The most recently calculated power consumption.
    double m_power;

    /// The available energy.
    EnergyTank m_energy_tank;

    /// The current state of the dynamics.
    State m_state;
};

} // FrankaRidgeback
