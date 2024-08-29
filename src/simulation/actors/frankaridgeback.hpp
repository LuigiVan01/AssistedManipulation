#pragma once

#include "controller/pid.hpp"
#include "controller/mppi.hpp"
#include "controller/energy.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"
#include "simulation/simulator.hpp"

/**
 * @brief 
 * 
 * Note that the simulator uses the PD controller with the feed-forward torque
 * control method. This is because the base and the gripper use proportional
 * derivative control for their joints, whereas the arm uses feed-forward torque
 * commands.
 * 
 * @BUG: It is not possible to disable the PD control of the arm joints,
 * including setting the PD gains of the arm to zero. Therefore the torque
 * commands are also opposing the PD controller.
 */
class FrankaRidgebackActor final : public Simulator::Actor
{
public:

    struct Configuration {

        /// Configuration of the mppi trajectory generator.
        mppi::Configuration mppi;

        /// The period of time between the controller updates.
        double controller_rate;

        /// The number of controller updates each update.
        unsigned int controller_substeps;

        /// The file name of the robot definition.
        std::string urdf_filename;

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

        // JSON conversion for franka ridgeback actor configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            controller_rate, controller_substeps, urdf_filename,
            end_effector_frame, initial_state, proportional_gain,
            differential_gain, energy
        )
    };

    /**
     * @brief Create a franka-ridgeback actor.
     * 
     * @param configuration The configuration of the actor.
     * @param simulator Pointer to the owning simulator.
     * 
     * @returns A pointer to the actor on success, or nullptr on failure.
     */
    static std::shared_ptr<FrankaRidgebackActor> create(
        const Configuration &configuration,
        Simulator *simulator,
        std::unique_ptr<mppi::Dynamics> &&dynamics,
        std::unique_ptr<mppi::Cost> &&cost,
        std::unique_ptr<mppi::Filter> &&filter = nullptr
    );

    /**
     * @brief Set the external end effector force for the next action only.
     * 
     * Keep calling this function to apply continuous forces.
     * 
     * @param force The force to apply to the end effector, in the world frame.
     */
    void set_end_effector_force(Eigen::Ref<Eigen::Vector3d> force);

    /**
     * @brief Set the external end effector torque for the next action only.
     * 
     * Keep calling this function to apply continuous torques.
     * 
     * @param torque The torque to apply to the end effector, in the world frame.
     */
    void set_end_effector_torque(Eigen::Ref<Eigen::Vector3d> torque);

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
    inline Eigen::Vector3d get_end_effector_position()
    {
        raisim::Vec<3> position;
        m_robot->getFramePosition(m_end_effector_frame_index, position);
        return position.e();
    }

    /**
     * @brief Get the end effector orientation.
     * @return The end effector orientation in world frame.
     */
    inline Eigen::Quaterniond get_end_effector_orientation()
    {
        raisim::Mat<3, 3> orientation;
        m_robot->getFrameOrientation(m_end_effector_frame_index, orientation);
        return Eigen::Quaterniond(orientation.e());
    }
 
    /**
     * @brief Get a pointer to the simulated articulated system.
     */
    inline raisim::ArticulatedSystem *get_robot() {
        return m_robot;
    }

    /**
     * @brief Get the current robot simulated state.
     */
    inline const FrankaRidgeback::State &get_state() const {
        return m_state;
    }

    /**
     * @brief Get the mppi trajectory generator.
     */
    inline const mppi::Trajectory &get_trajectory() const {
        return *m_trajectory;
    }

private:

    friend class Simulator;

    FrankaRidgebackActor(
        const Configuration &configuration,
        std::unique_ptr<mppi::Trajectory> &&controller,
        Simulator *simulator,
        raisim::ArticulatedSystem *robot,
        std::size_t end_effector_index,
        std::int64_t controller_countdown_max
    );

    /**
     * @brief Perform an action in the world.
     * @param handle The handle to the simulator to get data from.
     */
    void act(Simulator *simulator) override;

    /**
     * @brief Update the state of the frankaridgeback actor after acting.
     * @param simulator Pointer to the simulator to update state from.
     */
    void update(Simulator *simulator) override;

    /// The configuration of the actor.
    Configuration m_configuration;

    /// The owning simulator instance.
    Simulator *m_simulator;

    /// The trajectory generator.
    std::unique_ptr<mppi::Trajectory> m_trajectory;

    /// The simulated articulated object, generated from the urdf file.
    raisim::ArticulatedSystem *m_robot;

    /// The index of the end effector frame into raisim data structure.
    std::int64_t m_end_effector_frame_index;

    /// Countdown to next trajectory update.
    std::int64_t m_trajectory_countdown;

    /// The value to reset the countdown after update.
    std::int64_t m_trajectory_countdown_max;

    /// The simulated state.
    FrankaRidgeback::State m_state;

    /// The joint torques experienced due to the end effector force.
    Eigen::VectorXd m_external_joint_torques;

    /// The jacobian of the end effector frame.
    Eigen::MatrixXd m_end_effector_jacobian;

    /// The current control action.
    FrankaRidgeback::Control m_control;

    /// The available energy 
    EnergyTank m_energy_tank;
};
