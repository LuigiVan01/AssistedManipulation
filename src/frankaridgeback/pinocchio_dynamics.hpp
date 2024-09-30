#pragma once

#include <utility>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "controller/energy.hpp"
#include "controller/json.hpp"
#include "controller/forecast.hpp"
#include "frankaridgeback/dynamics.hpp"

namespace FrankaRidgeback {

/**
 * @brief Model predictive path integral control dynamics for the
 * frankaridgeback implemented with pinocchio.
 * 
 * @note See simulation/raisim_dynamics.hpp for a raisim based dynamics
 * implementation.
 * 
 * @warning This class is broken due to the pinocchio articulated body algorithm
 * function diverging. This class also expects torque control (not velocity
 * control).
 */
class PinocchioDynamics : public Dynamics
{
public:

    struct Configuration {

        /// The filename of the URDF robot definition file to instantiate the
        // model from.
        std::string filename;

        /// The name of the end effector frame.
        std::string end_effector_frame;

        /// The initial state of the dynamics.
        State initial_state;

        /// The initial energy in the energy tank.
        double energy;

        /// JSON conversion for dynamics configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            filename, end_effector_frame, energy
        );
    };

    /**
     * @brief Create a new franka ridgeback dynamics object.
     * 
     * Copies the configuration.
     * 
     * @param configuration The configuration of the dynamics.
     * @param wrench_forecast_handle An optional wrench estimation strategy.
     * 
     * @returns A pointer to the dynamics on success or nullptr on failure.
     */
    static std::unique_ptr<PinocchioDynamics> create(
        Configuration configuration,
        std::unique_ptr<Forecast::Handle> &&wrench_forecast_handle = nullptr
    );

    /**
     * @brief Copy the dynamics.
     */
    std::unique_ptr<mppi::Dynamics> copy() override;

    /**
     * @brief Get the configuration of the dynamics.
     */
    inline const Configuration &get_configuration()
    {
        return m_configuration;
    }

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     * 
     * @returns The FrankaRidgeback::State after stepping.
     */
    Eigen::Ref<VectorXd> step(const VectorXd &control, double dt) override;

    /**
     * @brief Set the dynamics simulation to a given state.
     * @param state The system state.
     * @param time The time of the dynamincs state.
     */
    void set_state(const VectorXd &state, double time) override;

    /**
     * @brief Get the dynamics state.
     */
    inline Eigen::Ref<VectorXd> get_state() override
    {
        return m_state;
    }

    /**
     * @brief Get the number of degrees of freedom for the frankaridgeback
     * control.
     */
    inline constexpr int get_control_dof() override
    {
        return DoF::CONTROL;
    }

    /**
     * @brief Get the number of degrees of freedom for the frankaridgeback
     * state.
     */
    inline constexpr int get_state_dof() override
    {
        return DoF::STATE;
    }

    /**
     * @brief Get the position of the end effector in world space.
     */
    Vector3d get_end_effector_position() const override
    {
        return m_data->oMf[m_end_effector_frame_index].translation();
    }

    /**
     * @brief Get the orientation of the end effector in world space.
     */
    Quaterniond get_end_effector_orientation() const override
    {
        return (Quaterniond)m_data->oMf[m_end_effector_frame_index].rotation();
    }

    /**
     * @brief Get the linear velocity (vx, vy, vz) of the end effector in world
     * space.
     */
    Vector3d get_end_effector_linear_velocity() const override
    {
        return m_end_effector_spatial_velocity.head<3>();
    }

    /**
     * @brief Get the angular velocity (wx, wy, wz) of the end effector in world
     * space.
     */
    Vector3d get_end_effector_angular_velocity() const override
    {
        return m_end_effector_spatial_velocity.tail<3>();
    }

    /**
     * @brief Get the linear acceleration (ax, ay, az) of the end effector in
     * world space.
     */
    Vector3d get_end_effector_linear_acceleration() const override
    {
        return m_end_effector_spatial_acceleration.head<3>();
    }

    /**
     * @brief Get the angular acceleration (alpha_x, alpha_y, alpha_z) of the
     * end effector in world space.
     */
    Vector3d get_end_effector_angular_acceleration() const override
    {
        return m_end_effector_spatial_acceleration.tail<3>();
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
    double get_power() const override
    {
        return m_power;
    }

    /**
     * @brief Get the current energy left in the energy tank.
     */
    double get_tank_energy() const override
    {
        return m_energy_tank.get_energy();
    }

    /**
     * @brief Get the end effector forecast wrench.
     * 
     * A prediction of the wrench applied to the end effector during rollout.
     * 
     * @returns The wrench (fx, fy, fz, tau_x, tau_y, tau_z) expected at the end
     * effector.
     */
    Vector6d get_end_effector_forecast_wrench() const override
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
    Vector6d get_end_effector_virtual_wrench() const override
    {
        return m_end_effector_virtual_wrench;
    }

    /**
     * @brief Get the actual wrench applied of the end effector by calls to
     * add_end_effector_true_wrench().
     * 
     * @todo Implement simulating external end effector wrench for pinocchio
     * dynamics.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @returns The actually applied wrench (fx, fy, fz, tau_x, tau_y, tau_z) at
     * the end effector in the world frame.
     */
    inline Vector6d get_end_effector_simulated_wrench() const override
    {
        return Vector6d::Zero();
    }

    /**
     * @brief Add cumulative wrench to the end effector, to be simulated on the
     * next step, after which it is set to zero.
     * 
     * @todo Implement simulating external end effector wrench for pinocchio
     * dynamics.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @param wrench The wrench to cumulative add to the end effector to be
     * simulated.
     */
    inline void add_end_effector_simulated_wrench(Vector6d wrench) override {}





















    /**
     * @brief Get the underlying pinocchio model.
     * @returns The underlying pinocchio model.
     */
    pinocchio::Model *get_model() const {
        return m_model.get();
    }

    /**
     * @brief Get the underlying pinocchio data.
     * 
     * See https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/topic/doc-v2/doxygen-html/structse3_1_1Data.html
     * for all available model data.
     * 
     * @return const pinocchio::Data* 
     */
    pinocchio::Data *get_data() const {
        return m_data.get();
    }

    /**
     * @brief Get the offset in the world space between two frames.
     * 
     * @param from_frame The first frame.
     * @param to_frame The second frame.
     * 
     * @returns The offset between the frames.
     */
    inline Vector3d get_frame_offset(
        const std::string &from_frame,
        const std::string &to_frame
    ){
        return m_data->oMf[m_model->getFrameId(to_frame)].translation() -
        m_data->oMf[m_model->getFrameId(from_frame)].translation();
    }

    /**
     * @brief Get the error screw error between two frames.
     * 
     * @param from_frame The first frame.
     * @param to_frame The second frame.
     * 
     * @returns The offset and orientation difference between the frames.
     */
    inline Vector6d get_frame_error(
        const std::string &from_frame,
        const std::string &to_frame
    ) {
        using namespace pinocchio;

        return log6(
            m_data->oMf[m_model->getFrameId(to_frame)].actInv(
                m_data->oMf[m_model->getFrameId(from_frame)]
            )
        ).toVector();
    }

    /**
     * @brief Get the pose of a frame.
     * 
     * @param frame 
     * @return std::tuple<Vector3d, Quaterniond> 
     */
    inline std::tuple<Vector3d, Quaterniond> get_frame_pose(
        const std::string &frame
    ){
        return std::make_tuple(
            m_data->oMf[m_model->getFrameId(frame)].translation(),
            (Quaterniond)m_data->oMf[m_model->getFrameId(frame)].rotation()
        );
    }

protected:

    /**
     * @brief Calculates kinematic information after position and velocity
     * updates.
     */
    void calculate();

    /**
     * @brief Calculate the effective end effector wrench based on the current.
     * 
     * @return Vector6d 
     */
    Vector6d calculate_virtual_end_effector_wrench();

    /**
     * @brief Initialise the dynamics parameters.
     * 
     * @param configuration The configuration of the dynamics.
     * @param model Pointer to the pinocchio model.
     * @param data Pointer to the pinocchio data.
     * @param geometry Pointer to the pinocchio geometry model.
     * @param end_effector_index Index into the vector of model joint frames for
     * the end effector.
     * @param wrench_forecast_handle An optional wrench estimation strategy.
     */
    PinocchioDynamics(
        const Configuration &configuration,
        std::unique_ptr<pinocchio::Model> &&model,
        std::unique_ptr<pinocchio::Data> &&data,
        std::unique_ptr<pinocchio::GeometryModel> &&geometry_model,
        std::unique_ptr<pinocchio::GeometryData> &&geometry_data,
        std::unique_ptr<Forecast::Handle> &&wrench_forecast_handle,
        std::size_t end_effector_index
    );

    /// The configuration of the pinocchio.
    Configuration m_configuration;

    /// The robot model.
    std::unique_ptr<pinocchio::Model> m_model;

    /// Data used for model calculations.
    std::unique_ptr<pinocchio::Data> m_data;

    /// Model used for collision detection.
    std::unique_ptr<pinocchio::GeometryModel> m_geometry_model;

    /// Data used for collision detection.
    std::unique_ptr<pinocchio::GeometryData> m_geometry_data;

    /// Index of the end effector frame in the frame vector.
    std::size_t m_end_effector_frame_index;

    /// The current joint positions.
    Eigen::Vector<double, DoF::JOINTS> m_joint_position;

    /// The current joint velocities.
    Eigen::Vector<double, DoF::JOINTS> m_joint_velocity;

    /// The current torques applied by the controller to the joints.
    Eigen::Vector<double, DoF::JOINTS> m_joint_torque;

    /// The acceleration of the joints from forward dynamics.
    Eigen::Vector<double, DoF::JOINTS> m_joint_acceleration;

    /// End effector jacobian in the world frame if enabled.
    Jacobian m_end_effector_jacobian;

    /// The velocity (vx, vy, vz, wx, wy, wz) of the end effector in the world
    /// frame.
    Vector6d m_end_effector_spatial_velocity;

    /// Get the acceleration of the end effector 
    Vector6d m_end_effector_spatial_acceleration;

    /// Optional pointer to the force predictor.
    std::unique_ptr<Forecast::Handle> m_forecast;

    /// A prediction of the wrench applied to the end effector during rollout.
    Vector6d m_end_effector_forecast_wrench;

    /// The virtual wrench is the wrench that would be applied to the end
    /// effector to produce its current linear and angular acceleration.
    Vector6d m_end_effector_virtual_wrench;

    /// The current power consumption.
    double m_power;

    /// The energy tank.
    EnergyTank m_energy_tank;

    /// The state to yield to the mppi trajectory generator.
    State m_state;

    /// Time of the last dynamics set.
    double m_time;
};

} // namespace FrankaRidgeback
