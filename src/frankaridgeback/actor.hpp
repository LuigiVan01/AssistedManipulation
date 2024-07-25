#pragma once

#include "controller/pid.hpp"
#include "frankaridgeback/simulator.hpp"
#include "controller/controller.hpp"

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

        /// The controller configuration.
        controller::Configuration controller;

        /// The period of time between the controller updates.
        double controller_rate;

        /// The number of updates of the controller each update.
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

    static std::shared_ptr<FrankaRidgebackActor> create(
        Configuration &&configuration,
        Simulator &simulator
    );

    /**
     * @brief Perform an action in the world.
     * @param handle The handle to the simulator to get data from.
     */
    void act(Simulator *simulator) override;

    /**
     * @brief Get a pointer to the simulated articulated system.
     */
    inline const raisim::ArticulatedSystem *robot() const {
        return m_robot;
    }

    /**
     * @brief Get the current robot simulated state.
     */
    FrankaRidgeback::State state();

private:

    FrankaRidgebackActor(
        Configuration &&configuration,
        std::unique_ptr<controller::Controller> &&controller,
        raisim::ArticulatedSystem *robot,
        std::int64_t controller_countdown_max
    );

    Configuration m_configuration;

    /// The simulated articulated object, generated from the urdf file.
    raisim::ArticulatedSystem *m_robot;

    std::unique_ptr<controller::Controller> m_controller;

    std::int64_t m_controller_countdown;

    std::int64_t m_controller_countdown_max;

    std::int64_t m_controller_substeps;

    /// The simulated state.
    FrankaRidgeback::State m_state;

    /// The current control action.
    FrankaRidgeback::Control m_control;
};
