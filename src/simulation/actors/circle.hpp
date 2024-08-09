#pragma once

#include <Eigen/Eigen>

#include "controller/pid.hpp"
#include "simulation/simulator.hpp"
#include "simulation/actors/circle.hpp"
#include "simulation/actors/frankaridgeback.hpp"

class RotatingPoint
{
public:

    struct Configuration {

        /// The origin of the circle to track.
        Eigen::Vector3d origin;

        /// The axis of rotation about the origin.
        Eigen::Vector3d axis;

        /// The radius of the circle.
        double radius;

        /// The angular velocity of the point to track.
        double angular_velocity;
    };

    inline RotatingPoint(const Configuration &configuration)
        : m_origin(configuration.origin)
        , m_axis(configuration.axis)
        , m_angular_velocity(configuration.angular_velocity)
    {
        // To initialise the point to rotate about the axis, need to get a
        // vector normal the the axis of rotation on the plane of rotation.
        // This offsets the axis of rotation slightly and projects the vector
        // onto the plane of rotation. To solve for the radius, normalise the
        // projected point and resize to the radius.

        const auto &axis = configuration.axis;

        Eigen::Vector3d offset = Eigen::Vector3d(1.0, 0.0, 0.0);

        // If the offset vector is in the direction of the axis, use a different
        // offset.
        if (axis.normalized().cwiseAbs().isApprox(offset.normalized().cwiseAbs()))
            offset = Eigen::Vector3d(0.0, 1.0, 0.0);

        auto to_project = axis + offset;

        // Project the point to the plane of rotation.
        Eigen::Vector3d projected = (
            to_project - axis.dot(to_project) / axis.dot(axis) * axis
        );

        // Normalise and extend to the required radius.
        m_point = projected.normalized() * configuration.radius;
    }

    inline Eigen::Vector3d operator()(double time) const
    {
        return m_origin + Eigen::AngleAxisd(
            time * m_angular_velocity,
            m_axis
        ) * m_point;
    }

private:

    /// The point to rotate about the axis.
    Eigen::Vector3d m_point;

    /// The origin of the circle to track, to, add to the point.
    Eigen::Vector3d m_origin;

    /// The axis of rotation about the origin.
    Eigen::Vector3d m_axis;

    /// The angular velocity of the point to track.
    double m_angular_velocity;
};

class CircleActor final : public Simulator::Actor
{
public:

    struct Configuration {

        RotatingPoint::Configuration rotating_point;

        /// The pid behaviour of keeping the end effector at the point.
        controller::PID::Configuration pid;

        /// Pointer to the simulated system to apply force to.
        FrankaRidgebackActor *robot;
    };

    /**
     * @brief Create a new actor.
     * 
     * @param configuration The point actor configuration.
     * 
     * @returns A pointer to the actor on success, or nullptr if the configured
     * frame does not exist.
     */
    static inline std::shared_ptr<CircleActor> create(
        const Configuration &configuration,
        Simulator *simulator
     ) {
        if (configuration.robot == nullptr) {
            std::cerr << "point actor robot pointer is nullptr" << std::endl;
            return nullptr;
        }

        return std::shared_ptr<CircleActor>(
            new CircleActor(configuration, simulator)
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

        auto position = m_configuration.robot->get_end_effector_position();

        // Update the pid controller.
        m_pid.set_reference(reference);
        m_pid.update(position, simulator->get_time());

        // Apply the pid controller for to the end effector.
        Eigen::Vector3d control = m_pid.get_control();
        m_configuration.robot->set_end_effector_force(control);
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
        Simulator *simulator
      ) : m_configuration(configuration)
        , m_rotating_point(configuration.rotating_point)
        , m_pid(configuration.pid)
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

    /// Visual sphere of the point being tracked.
    raisim::Visuals *m_tracking_sphere;
};
