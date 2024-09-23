#pragma once

#include <tuple>

#include "controller/eigen.hpp"
#include "controller/json.hpp"

/**
 * @brief Base class for implementing a positional trajectory.
 */
class PointTrajectory
{
public:

    /**
     * @brief Get the position of the trajectory at a given time.
     * 
     * @param time 
     * @return Vector3d 
     */
    virtual Vector3d get_position(double time) = 0;

protected:

    PointTrajectory() = default;
};

/**
 * @brief Base class for implementing an orientation trajectory.
 */
class OrientationTrajectory
{
public:

    /**
     * @brief Get the orientation at a given time.
     * 
     * @param time The time of the orientation.
     * @returns The orientation in world space of the trajectory.
     */
    virtual Quaterniond get_orientation(double time) = 0;

protected:

    OrientationTrajectory() = default;
};

/**
 * @brief A trajectory that stays in the same position forever.
 */
class PointTrajectory : public PointTrajectory
{
public:

    struct Configuration {

        /// The point of the trajectory.
        Vector3d point;

        /// JSON conversion for point trajectory configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, point)
    };

    /**
     * @brief Initialise a point trajectory.
     * @param configuration The configuration of the point trajectory.
     */
    inline PointTrajectory(const Configuration &configuration)
        : m_point(configuration.point)
    {}

    /**
     * @brief Get the point of the trajectory.
     * @param time Unused.
     */
    inline Vector3d get_position(double time) override
    {
        return m_point;
    }

private:

    /// The point of the trajectory.
    Vector3d m_point;
};

/**
 * @brief A point trajectory that rotates in a circle.
 */
class CircleTrajectory : public PointTrajectory
{
public:

    struct Configuration {

        /// The origin of the circle to track.
        Vector3d origin;

        /// The axis of rotation about the origin.
        Vector3d axis;

        /// The radius of the circle.
        double radius;

        /// The angular velocity of the point to track.
        double angular_velocity;

        // JSON conversion for rotating point configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            origin, axis, radius, angular_velocity
        )
    };

    inline CircleTrajectory(const Configuration &configuration)
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

        Vector3d offset = Vector3d(1.0, 0.0, 0.0);

        // If the offset vector is in the direction of the axis, use a different
        // offset.
        if (axis.normalized().cwiseAbs().isApprox(offset.normalized().cwiseAbs()))
            offset = Vector3d(0.0, 1.0, 0.0);

        auto to_project = axis + offset;

        // Project the point to the plane of rotation.
        Vector3d projected = (
            to_project - axis.dot(to_project) / axis.dot(axis) * axis
        );

        // Normalise and extend to the required radius.
        m_point = projected.normalized() * configuration.radius;
    }

    /**
     * @brief Get the position around the circle at the given time.
     * 
     * @param time The time to get the circle position.
     * @returns The position around the circle.
     */
    inline Vector3d get_position(double time) override
    {
        return m_origin + Eigen::AngleAxisd(
            time * m_angular_velocity,
            m_axis
        ) * m_point;
    }

private:

    /// The point to rotate about the axis.
    Vector3d m_point;

    /// The origin of the circle to track, to, add to the point.
    Vector3d m_origin;

    /// The axis of rotation about the origin.
    Vector3d m_axis;

    /// The angular velocity of the point to track.
    double m_angular_velocity;
};

/**
 * @brief A point-wise trajectory following a 3D lissajous curve.
 * 
 * A lissajous curve is parameterised by:
 * 
 *      x = a * sin(t)
 *      y = b * sin(nu * t + phi)
 *      z = c * sin(gamma * t + psi)
 * 
 * For the configuration:
 * 
 *      x = x_amplitude * sin(t)
 *      y = y_amplitude * sin(y_frequency * t + y_phase)
 *      z = z_amplitude * sin(z_frequencz * t + z_phase)
 */
class LissajousTrajectory
{
public:

    struct Configuration {

        /// Origin of the lissajous trajectory in space.
        Vector3d origin;

        /// Amplitude of the sine curve in the x world axis.
        double x_amplitude;

        /// Amplitude of the sine curve in the y world axis.
        double y_amplitude;

        /// Amplitude of the sine curve in the z world axis.
        double z_amplitude;

        /// Frequency of the sine curve in the y world axis.
        double y_frequency;

        /// Frequency of the sine curve in the z world axis.
        double z_frequency;

        /// Phase of the sine curve in the y world axis.
        double y_phase;

        /// Phase of the sine curve in the z world axis.
        double z_phase;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            origin, x_amplitude, y_amplitude, z_amplitude, y_frequency,
            z_frequency, y_phase, z_phase
        )
    };

    /**
     * @brief Initialise a new lissajous trajectory.
     * @param configuration The lissajous trajectory configuration.
     */
    inline LissajousTrajectory(const Configuration &configuration)
        : m_configuration(configuration)
    {}

    /**
     * @brief Get the position object
     * 
     * @param time 
     * @return Vector3d 
     */
    inline Vector3d get_position(double time) override
    {
        return m_configuration.origin + Vector3d{
            m_configuration.x_amplitude * std::sin(time),
            m_configuration.y_amplitude * std::sin(
                m_configuration.y_frequency * time + m_configuration.y_phase
            ),
            m_configuration.z_amplitude * std::sin(
                m_configuration.z_frequencz * time + m_configuration.z_phase
            )
        };
    }

private:

    /// The configuration of the lissajous trajectory.
    Configuration m_configuration;
};

class RectangleTrajectory : public PointTrajectory
{
public:

    struct Configuration {

    };

private:

};

struct PointTrajectory::Configuration {

};

