#pragma once

#include "dynamics.hpp"

struct Configuration {
    double timestep;
};

/**
 * @brief Raisim simulator.
 * lib\sampling_based_control\mppi_examples\mppi_manipulation\src\dynamics.cpp
 */
class Simulator
{
public:

    /**
     * @brief Create a new instance of the simulator.
     * 
     * @param urdf_filename The filename of the robot definition file to load.
     * 
     * @returns A pointer to the simulator on success, or nullptr on failure.
     */
    std::unique_ptr<Simulator> create(std::string urdf_filename);

    /**
     * @brief Step the simulation, during which the control action is applied.
     * 
     * @param control 
     * @param dt 
     */
    void step(FrankaRidgeback::Control control, double dt);

private:

    Simulator() = default;

    Eigen::VectorXd m_position_control;

    Eigen::VectorXd m_velocity_control;

    FrankaRidgeback::State m_state;

    raisim::World m_world;

    raisim::ArticulatedSystem *m_robot;

    raisim::RaisimServer m_server;
};

} // namespace simulator
