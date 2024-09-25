#pragma once

#include <Eigen/Eigen>

#include "controller/json.hpp"
#include "controller/pid.hpp"
#include "simulation/simulator.hpp"
#include "simulation/actors/circle.hpp"
#include "simulation/actors/frankaridgeback.hpp"

class CircleActor final : public Simulator::Actor
{
public:

    struct Configuration {

        RotatingPoint::Configuration rotating_point;

        /// The pid behaviour of keeping the end effector at the point.
        controller::PID::Configuration pid;

        // JSON conversion for circle actor configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            rotating_point, pid
        )
    };

    /**
     * @brief Create a new actor.
     * 
     * @param configuration The point actor configuration.
     * @param simulator Pointer to the simulator
     * @param robot Pointer to the simulated system to apply force to.
     * 
     * @returns A pointer to the actor on success, or nullptr if the configured
     * frame does not exist.
     */
    static inline std::shared_ptr<CircleActor> create(
        const Configuration &configuration,
        Simulator *simulator,
        FrankaRidgeback::Actor *robot
     ) {
        if (robot == nullptr) {
            std::cerr << "point actor robot pointer is nullptr" << std::endl;
            return nullptr;
        }

        return std::shared_ptr<CircleActor>(
            new CircleActor(configuration, simulator, robot)
        );
    }

    /**
     * @brief Applies a force to the robot end effector towards the point.
     * 
     * @param world Pointer to the simulated world.
     * @param robot Pointer to the robot.
     * @param time The time in the simulation.
     */
    inline void act(Simulator *simulator) override
    {
        // Get the current position of the rotating point to track.
        Eigen::Vector3d reference = m_rotating_point(simulator->get_time());

        // Display the new position in the simulator.
        m_tracking_sphere->setPosition(reference);

        auto position = m_robot->get_dynamics()->get_end_effector_position();

        // Update the pid controller.
        m_pid.set_reference(reference);
        m_pid.update(position, simulator->get_time());

        // Apply the pid controller for to the end effector.
        Eigen::Vector<double, 6> control = m_pid.get_control();
        m_robot->get_dynamics()->add_end_effector_true_wrench(control);
    }

    inline void update(Simulator *simulator) override {}

    /**
     * @brief Get the pid controller used to track the point.
     */
    inline controller::PID &get_pid() {
        return m_pid;
    }

private:

    CircleActor(
        const Configuration &configuration,
        Simulator *simulator,
        FrankaRidgeback::Actor *robot
      ) : m_configuration(configuration)
        , m_rotating_point(configuration.rotating_point)
        , m_pid(configuration.pid)
        , m_robot(robot)
    {
        Eigen::Vector3d reference = m_rotating_point(configuration.pid.time);
        m_pid.set_reference(reference);

        auto position = m_rotating_point(configuration.pid.time);

        // Add a visual sphere to show the tracked point.
        m_tracking_sphere = simulator->get_server().addVisualSphere("tracking_sphere", 0.05);
        m_tracking_sphere->setPosition(position);
    }

    /// Configuration of the point tracking actor.
    Configuration m_configuration;

    /// The point rotating in space to force to.
    RotatingPoint m_rotating_point;

    /// A pid controller to move towards the point.
    controller::PID m_pid;

    /// Pointer to the robot.
    FrankaRidgeback::Actor *m_robot;

    /// Visual sphere of the point being tracked.
    raisim::Visuals *m_tracking_sphere;
};
