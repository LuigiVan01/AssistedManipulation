#pragma once

#include "controller/pid.hpp"
#include "controller/mppi.hpp"
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

        /// The period of time between the controller updates.
        double controller_rate;

        /// The number of controller updates each update.
        std::int64_t controller_substeps;

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
        mppi::Trajectory::Configuration &&mppi,
        Configuration &&configuration,
        Simulator *simulator
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

    inline const mppi::Trajectory &get_trajectory() const {
        return *m_trajectory;
    }

private:

    friend class Simulator;

    FrankaRidgebackActor(
        Configuration &&configuration,
        Simulator *simulator,
        std::unique_ptr<mppi::Trajectory> &&controller,
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

    void get_end_effector_jacobian(Eigen::MatrixXd &J);

    /**
     * @brief Get the wrench on the end effectors body frame.
     * @returns The external wrench. 
     */
    void get_end_effector_wrench(Eigen::VectorXd &wrench);

    Configuration m_configuration;

    Simulator *m_simulator;

    std::unique_ptr<mppi::Trajectory> m_trajectory;

    /// The simulated articulated object, generated from the urdf file.
    raisim::ArticulatedSystem *m_robot;

    std::int64_t m_end_effector_frame_index;

    std::int64_t m_trajectory_countdown;

    std::int64_t m_trajectory_countdown_max;

    /// The simulated state.
    FrankaRidgeback::State m_state;

    bool m_external_force_applied;

    Eigen::VectorXd m_external_joint_torques;

    Eigen::MatrixXd m_contact_jacobian;

    /// The current control action.
    FrankaRidgeback::Control m_control;
};
