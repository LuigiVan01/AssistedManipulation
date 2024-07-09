#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "frankaridgeback/state.hpp"
#include "frankaridgeback/control.hpp"

/**
 * @brief Raisim simulator.
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
class Simulator
{
public:

    struct Configuration {

        // The file name of the robot definition.
        std::string urdf_filename;

        // The timestep of the simulation.
        double timestep;

        // The gravitational acceleration.
        Eigen::Vector3d gravity;

        // The initial state.
        FrankaRidgeback::State initial_state;

        // The proportional gain of the joint PD controller.
        FrankaRidgeback::Control proportional_gain;

        // The differential gain of the joint PD controller.
        FrankaRidgeback::Control differential_gain;
    };

    ~Simulator();

    /**
     * @brief Create a new instance of the simulator.
     * 
     * @param urdf_filename The filename of the robot definition file to load.
     * 
     * @returns A pointer to the simulator on success, or nullptr on failure.
     */
    static std::unique_ptr<Simulator> create(const Configuration &configuration);

    /**
     * @brief Step the simulation with a control action.
     * @param control The control parameters to apply to the robot.
     */
    const FrankaRidgeback::State &step(FrankaRidgeback::Control &control);

    /**
     * @brief Reset the simulator to the initial state.
     */
    inline void set() {
        set(m_configuration.initial_state);
    }

    /**
     * @brief Reset the simulator to a state.
     * @param state The state to reset to.
     */
    void set(FrankaRidgeback::State &state);

    /**
     * @brief Get the current state in the simulation.
     * @returns The current state.
     */
    inline const FrankaRidgeback::State &state() {
        return m_state;
    }

    /**
     * @brief Get the time in the simulation.
     * @returns The time elapsed in seconds.
     */
    inline double time() {
        return m_time;
    }

private:

    Simulator(
        const Configuration &configuration,
        std::unique_ptr<raisim::World> &&world,
        raisim::ArticulatedSystem *robot
    );

    Configuration m_configuration;

    FrankaRidgeback::State m_state;

    double m_time;

    std::unique_ptr<raisim::World> m_world;

    raisim::ArticulatedSystem *m_robot;

    raisim::RaisimServer m_server;
};
