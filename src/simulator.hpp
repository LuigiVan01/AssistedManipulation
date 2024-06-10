#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "dynamics.hpp"

/**
 * @brief Raisim simulator.
 * lib\sampling_based_control\mppi_examples\mppi_manipulation\src\dynamics.cpp
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
        double gravity;

        // The initial state.
        FrankaRidgeback::State initial_state;
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
    inline void reset() {
        reset(m_configuration.initial_state);
    }

    /**
     * @brief Reset the simulator to a state.
     * @param state The state to reset to.
     */
    void reset(FrankaRidgeback::State &state);

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

    Eigen::VectorXd m_position_control;

    Eigen::VectorXd m_velocity_control;

    FrankaRidgeback::State m_state;

    double m_time;

    std::unique_ptr<raisim::World> m_world;

    raisim::ArticulatedSystem *m_robot;

    raisim::RaisimServer m_server;
};
