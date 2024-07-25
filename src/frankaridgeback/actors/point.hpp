#pragma once

#include <iostream>

#include "controller/pid.hpp"
#include "frankaridgeback/simulator.hpp"

/**
 * @brief An actor that attempts to force a frame to a point in space using PID.
 */
class PointActor final : public Simulator::Actor
{
public:

    struct Configuration {

        /// The point to maintain the end effector at.
        Eigen::Vector3d point;

        /// The pid behaviour of keeping the end effector at the point.
        controller::PID::Configuration pid;

        /// Pointer to the simulated system to apply force to.
        raisim::ArticulatedSystem *robot;

        /// The frame to move to the point.
        std::string frame;

        /// The initial pid time, for calculating first derivative.
        double initial_time;
    };

    /**
     * @brief Create a new actor.
     * 
     * @param configuration The point actor configuration.
     * 
     * @returns A pointer to the actor on success, or nullptr if the configured
     * frame does not exist.
     */
    inline std::shared_ptr<PointActor> create(
        const Configuration &configuration,
        Simulator *simulator
     ) {
        if (configuration.robot == nullptr) {
            std::cerr << "point actor robot pointer is nullptr" << std::endl;
            return nullptr;
        }

        auto frame_index = configuration.robot->getFrameIdxByName(
            configuration.frame
        );

        if (frame_index > configuration.robot->getFrames().size()) {
            std::cerr << "point actor frame " << configuration.frame
                      << " does not exist in " << configuration.robot->getName()
                      << std::endl;
            return nullptr;
        }

        auto sphere = simulator->get_server().addVisualSphere("sphere", 0.1);
        sphere->setPosition(configuration.point);

        return std::shared_ptr<PointActor>(new PointActor(configuration));
    }

    /**
     * @brief Applies a force to the robot end effector towards the point.
     * 
     * @param world Pointer to the simulated world.
     * @param robot Pointer to the robot.
     * @param time The time in the simulation.
     */
    inline void act(Simulator::Handle *simulator) override
    {
        Eigen::Vector3d position = m_configuration.robot->getFrames()[m_frame_index].position;
        m_pid.update(position, m_force, simulator->get_time());
        m_configuration.robot->setExternalForce(m_configuration.frame, m_force);
    }

    /**
     * @brief Get the applied force by the actor.
     */
    inline Eigen::VectorXd force() {
        return m_force;
    }

    /**
     * @brief Get the pid controller used to track the point.
     */
    inline controller::PID &pid() {
        return m_pid;
    }

private:

    PointActor(const Configuration &configuration)
        : m_configuration(configuration)
        , m_pid(configuration.pid, configuration.point, configuration.initial_time)
        , m_force(Eigen::Vector3d::Zero())
    {}

    /// Configuration of the point tracking actor.
    Configuration m_configuration;

    /// Index into the articulated system to the frame to be forced.
    unsigned int m_frame_index;

    /// A pid controller to move towards the point.
    controller::PID m_pid;

    /// The last force control applied to the point.
    Eigen::Vector3d m_force;
};
