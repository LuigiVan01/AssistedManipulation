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
     * @param configuration The configuration of the dynamics.
     * @param force_predictor An optional force estimation strategy.
     * 
     * @returns A pointer to the dynamics on success or nullptr on failure.
     */
    static std::unique_ptr<Dynamics> create(
        const Configuration &configuration,
        Forecast *force_predictor = nullptr
    );

    /**
     * @brief Copy the dynamics.
     */
    std::unique_ptr<mppi::Dynamics> copy() override;

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) override;

    /**
     * @brief Set the dynamics simulation to a given state.
     * @param state The system state.
     * @param time The time of the dynamincs state.
     */
    void set_state(const Eigen::VectorXd &state, double time) override;

    /**
     * @brief Get the dynamics state.
     */
    inline Eigen::Ref<Eigen::VectorXd> get_state() override
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
    virtual Vector3d get_end_effector_position()
    {
        return m_data->oMf[m_end_effector_frame_index].translation();
    }

    /**
     * @brief Get the orientation of the end effector in world space.
     */
    virtual Quaterniond get_end_effector_orientation()
    {
        return m_data->oMf[m_end_effector_frame_index].rotation();
    }

    /**
     * @brief Get the linear velocity (vx, vy, vz) of the end effector in world
     * space.
     */
    virtual Vector3d get_end_effector_linear_velocity()
    {
        return m_end_effector_spatial_velocity.head<3>();
    }

    /**
     * @brief Get the angular velocity (wx, wy, wz) of the end effector in world
     * space.
     */
    virtual Vector3d get_end_effector_angular_velocity()
    {
        return m_end_effector_spatial_velocity.tail<3>();
    }

    /**
     * @brief Get the linear acceleration (ax, ay, az) of the end effector in
     * world space.
     */
    virtual Vector3d get_end_effector_linear_acceleration()
    {
        return ;
    }

    /**
     * @brief Get the angular acceleration (alpha_x, alpha_y, alpha_z) of the
     * end effector in world space.
     */
    virtual Vector3d get_end_effector_angular_acceleration()
    {
        return ;
    }

    /**
     * @brief Get the jacobian of the end effector.
     */
    virtual Eigen::Ref<Jacobian> get_end_effector_jacobian()
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
    virtual double get_power()
    {
        return ;
    }

    /**
     * @brief Get the end effector forecast wrench.
     * 
     * A prediction of the wrench applied to the end effector during rollout.
     * 
     * @returns The wrench (fx, fy, fz, tau_x, tau_y, tau_z) expected at the end
     * effector.
     */
    virtual Vector6d get_end_effector_forecast_wrench()
    {
        return ;
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
    virtual Vector6d get_end_effector_virtual_wrench()
    {
        return ;
    }






















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
    inline Eigen::Vector3d get_frame_offset(
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
    inline Eigen::Matrix<double, 6, 1> get_frame_error(
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
     * @return std::tuple<Eigen::Vector3d, Eigen::Quaterniond> 
     */
    inline std::tuple<Eigen::Vector3d, Eigen::Quaterniond> get_frame_pose(
        const std::string &frame
    ){
        return std::make_tuple(
            m_data->oMf[m_model->getFrameId(frame)].translation(),
            (Eigen::Quaterniond)m_data->oMf[m_model->getFrameId(frame)].rotation()
        );
    }

private:

    /**
     * @brief Calculates kinematic information after position and velocity
     * updates.
     */
    void update_kinematics();

    /**
     * @brief Initialise the dynamics parameters.
     * 
     * @param model Pointer to the pinnochio model.
     * @param data Pointer to the pinnochio data.
     * @param end_effector_index Index into the vector of model joint frames for
     * the end effector.
     * @param force_predictor An optional force estimation strategy.
     * @param energy The initial energy in the energy tank.
     */
    PinocchioDynamics(
        std::unique_ptr<pinocchio::Model> &&model,
        std::unique_ptr<pinocchio::Data> &&data,
        std::unique_ptr<Forecast::Handle> &&force_predictor_handle,
        std::size_t end_effector_index,
        double energy
    );

    /// The robot model.
    std::unique_ptr<pinocchio::Model> m_model;

    /// Data used for model calculations.
    std::unique_ptr<pinocchio::Data> m_data;

    /// Index of the end effector frame in the frame vector.
    std::size_t m_end_effector_frame_index;

    /// Optional pointer to the force predictor.
    std::unique_ptr<Forecast::Handle> m_forecast;

    /// End effector jacobian in the world frame if enabled.
    Jacobian m_end_effector_jacobian;

    /// The velocity (vx, vy, vz, wx, wy, wz) of the end effector in the world
    /// frame.
    Vector6d m_end_effector_spatial_velocity;

    /// Get the acceleration of the end effector 
    Vector6d m_end_effector_spatial_acceleration;

    /// The energy tank.
    EnergyTank m_energy_tank;

    /// The state to yield to the mppi trajectory generator.
    State m_state;

    /// Time of the last dynamics set.
    double m_time;
};

} // namespace FrankaRidgeback
