#pragma once

#include "controller/eigen.hpp"
#include "controller/mppi.hpp"
#include "controller/forecast.hpp"
#include "frankaridgeback/dof.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

class DynamicsForecast;

/**
 * @brief Matrix mapping of the end effector velocity to the joint velocities.
 */
using Jacobian = Eigen::Matrix<double, 6, DoF::JOINTS>;

/**
 * @brief Data structure containing frame kinematic information at a given time.
 */
struct EndEffectorState {

    /// Position (x, y, z).
    Vector3d position = Vector3d::Zero();

    /// Orientation.
    Quaterniond orientation = Quaterniond::Identity();

    /// The linear velocity (vx, vy, vz).
    Vector3d linear_velocity = Vector3d::Zero();

    /// The angular velocity (wx, wy, wz).
    Vector3d angular_velocity = Vector3d::Zero();

    /// The linear acceleration (ax, ay, az).
    Vector3d linear_acceleration = Vector3d::Zero();

    /// The angular acceleration (alpha_x, alpha_y, alpha_z).
    Vector3d angular_acceleration = Vector3d::Zero();

    /// The jacobian of the end effector in the world frame.
    Jacobian jacobian = Jacobian::Zero();
};

/**
 * @brief Base class for all frankaridgeback mppi dynamics implementations.
 * 
 * Used to ensure the derived dynamics class provides methods used by objective
 * functions.
 */
class Dynamics : public mppi::Dynamics
{
public:

    /**
     * @brief Static function that returns the path to the dynamics urdf file.
     * @returns Path to the robot definition.
     */
    static inline std::filesystem::path find_path() {
        return (std::filesystem::current_path() / "model/robot.urdf").string();
    }

    /**
     * @brief Get the kinematics of the end effector.
     */
    virtual const EndEffectorState &get_end_effector_state() const = 0;

    /**
     * @brief Get the current dynamics power usage.
     * 
     * This is given by the sum of generalised joint force multiplied by their
     * generalised velocities. This is torque * angular velocity for revolute
     * joints and force * linear velocity for prismatic joints.
     * 
     * @return The current power usage in joules/s.
     */
    virtual double get_power() const = 0;

    /**
     * @brief Get the current energy left in the energy tank.
     */
    virtual double get_tank_energy() const = 0;

    /**
     * @brief Get the actual wrench applied of the end effector by calls to
     * add_end_effector_true_wrench().
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @returns The actually applied wrench (fx, fy, fz, tau_x, tau_y, tau_z) at
     * the end effector in the world frame.
     */
    virtual Vector6d get_end_effector_simulated_wrench() const = 0;

    /**
     * @brief Add cumulative wrench to the end effector, to be simulated on the
     * next step, after which it is set to zero.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @param wrench The wrench to cumulative add to the end effector to be
     * simulated.
     */
    virtual void add_end_effector_simulated_wrench(Vector6d wrench) = 0;
};

/**
 * @brief A forecast of the dynamics from an external wrench.
 */
class DynamicsForecast
{
public:

    struct Configuration {

        /// The time step of the forecasted dynamics trajectory.
        double time_step;

        /// The time horison over which to forecast the dynamics trajectory.
        double horison;
    };

    /**
     * @brief Get a handle to the dynamics forecast.
     */
    class Handle
    {
    public:

        /**
         * @brief Forecast at a time in the future.
         * 
         * @returns The predicted state.
         */
        inline DynamicsForecast *get()
        {
            return m_parent;
        };

        /**
         * @brief Make a copy of the handle.
         */
        inline std::unique_ptr<Handle> copy()
        {
            return std::unique_ptr<Handle>(new Handle(m_parent));
        };

    private:

        friend class DynamicsForecast;

        Handle(DynamicsForecast *parent)
            : m_parent(parent)
        {}

        /// Pointer to the owning dynamics forecast instance.
        DynamicsForecast *m_parent;
    };

    /**
     * @brief Create a new instance of the dynamics.
     * 
     * @param configuration The configuration of the dynamics forecast.
     * @param dynamics The dynamics instance to use to forecast.
     * 
     * @returns A pointer to the dynamics forecast on success or nullptr on
     * failure.
     */
    static std::unique_ptr<DynamicsForecast> create(
        const Configuration &configuration,
        std::unique_ptr<Dynamics> &&dynamics,
        std::unique_ptr<Forecast> &&wrench_forecast
    );

    /**
     * @brief Create a handle to the dynamics forecast.
     * @returns A pointer to the handle on success or nullptr on failure.
     */
    inline std::unique_ptr<Handle> create_handle()
    {
        return std::unique_ptr<Handle>(new Handle(this));
    }

    /**
     * @brief Update the forecasted wrench with an observation.
     * 
     * @param wrench The wrench on the end effector observed.
     * @param time The time of the observation.
     */
    inline void observe(Vector6d &wrench, double time)
    {
        m_forecast->update(wrench, time);
    }

    /**
     * @brief Update the forecasted wrench with an observation of time.
     * 
     * @param time The time of the observation.
     */
    inline void observe(double time)
    {
        m_forecast->update(time);
    }

    /**
     * @brief Update the dynamics forecast over the time horison based on
     * the observed wrench trajectory.
     * 
     * @param state The initial state to forecast the dynamics from.
     * @param time The time of the initial state.
     */
    void forecast(State state, double time);

    /**
     * @brief Get the kinematics of the end effector.
     * @param time The time of the end effector state.
     */
    const EndEffectorState &get_end_effector_state(double time);

    /**
     * @brief Get the end effector forecast wrench.
     * 
     * A prediction of the wrench applied to the end effector during rollout.
     * 
     * @returns The wrench (fx, fy, fz, tau_x, tau_y, tau_z) expected at the end
     * effector.
     */
    Vector6d get_end_effector_wrench(double time) const;

    double get_time_step() const
    {
        return m_configuration.time_step;
    }

    double get_horison() const
    {
        return m_configuration.horison;
    }

    const std::vector<EndEffectorState> &get_end_effector_trajectory() const
    {
        return m_end_effector;
    }

    const std::vector<Vector6d> &get_external_wrench_trajectory() const
    {
        return m_end_effector_wrench;
    }

protected:

    /**
     * @brief Initialise the dynamics forecast.
     * 
     * @param configuration The configuration of the forecast.
     * @param dynamics The dynamics to use to forecast.
     * @param wrench_forecast The forecast strategy used to forecast the wrench
     * applied to the end effector.
     * @param steps The number of time steps in the time horison.
     */
    DynamicsForecast(
        const Configuration &configuration,
        std::unique_ptr<Dynamics> &&dynamics,
        std::unique_ptr<Forecast> &&wrench_forecast,
        unsigned int steps
    );

    /// The configuration of the dynamics forecast.
    Configuration m_configuration;

    /// The number of time steps in the horison.
    const unsigned int m_steps;

    /// The time of the last forecast.
    double m_last_forecast;

    /// The dynamics used to rollout the trajectory given the forecasted wrench.
    std::unique_ptr<Dynamics> m_dynamics;

    /// Pointer to the forecast wrench.
    std::unique_ptr<Forecast> m_forecast;

    /// The forecasted end effector trajectory.
    std::vector<EndEffectorState> m_end_effector;

    /// The forecasted wrench.
    std::vector<Vector6d> m_end_effector_wrench;
};

} // namespace FrankaRidgeback
