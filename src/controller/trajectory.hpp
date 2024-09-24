#pragma once

#include <cmath>
#include <tuple>
#include <iostream>

#include "controller/eigen.hpp"
#include "controller/json.hpp"

/**
 * @brief Base class for implementing a positional trajectory.
 */
class PositionTrajectory
{
public:

    struct Configuration;

    /// Virtual destructor for derived classes.
    virtual ~PositionTrajectory() {};

    /**
     * @brief Create a configuration trajectory.
     * 
     * @param configuration The configuration of the positional trajectory.
     * @returns The position trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<PositionTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position of the trajectory at a given time.
     * 
     * @param time The time of the position.
     * @returns The position. 
     */
    virtual Vector3d get_position(double time) = 0;

protected:

    PositionTrajectory() = default;
};

/**
 * @brief Base class for implementing an orientation trajectory.
 */
class OrientationTrajectory
{
public:

    /// Virtual destructor for derived classes.
    virtual ~OrientationTrajectory();

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
class PointTrajectory : public PositionTrajectory
{
public:

    struct Configuration {

        /// The point of the trajectory.
        Vector3d point;

        /// JSON conversion for point trajectory configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, point)
    };

    /**
     * @brief Create a new point trajectory.
     * 
     * @param configuration The configuration of the point trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<PointTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position of the trajectory at a given time.
     * 
     * @param time Unused.
     * @returns The position. 
     */
    Vector3d get_position(double time) override;

private:

    /**
     * @brief Initialise a point trajectory.
     * @param configuration The configuration of the point trajectory.
     */
    PointTrajectory(const Configuration &configuration);

    /// The point of the trajectory.
    Vector3d m_point;
};

/**
 * @brief A point trajectory that rotates in a circle.
 */
class CircularTrajectory : public PositionTrajectory
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

    /**
     * @brief Create a new circular trajectory.
     * 
     * @param configuration The configuration of the circular trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<CircularTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position of the trajectory at a given time.
     * 
     * @param time The time of the position.
     * @returns The position around the circle. 
     */
    inline Vector3d get_position(double time) override;

private:

    CircularTrajectory(const Configuration &configuration);

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
 * @brief A trajectory that moves in a rectangle on a plane in space.
 */
class RectangularTrajectory : public PositionTrajectory
{
public:

    struct Configuration {

        /// The origin of the rectangle to track.
        Vector3d origin;

        /// The axis defining the orientation of the plane 
        Vector3d axis;

        // The angle to rotate the rectangle on the plane.
        double angle;

        /// Width of the rectangle along the x axis of the plane.
        double width;

        /// Height of the rectangle along the y axis of the plane.
        double height;

        /// The velocity of the trajectory.
        double velocity;

        /// JSON conversion for rectangular trajectory configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            origin, axis, angle, width, height, velocity
        );
    };

    /**
     * @brief Create a new circular trajectory.
     * 
     * @param configuration The configuration of the circular trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<RectangularTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position around the rectangle.
     * 
     * @todo Make this nicer.
     * 
     * @param time The time of the position.
     * @return The position. 
     */
    inline Vector3d get_position(double time) override;

private:

    RectangularTrajectory(const Configuration &configuration);

    /// The configuration of the rectangular trajectory.
    Configuration m_configuration;

    /// The total circumference of the rectangle on the 2D plane.
    double m_circumference;

    /// Transformation from the 2D plane to the 3D plane.
    Eigen::Transform<double, 3, Eigen::Affine>  m_transform;
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
class LissajousTrajectory : public PositionTrajectory
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
     * @brief Create a new circular trajectory.
     * 
     * @param configuration The configuration of the circular trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<LissajousTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position of the lissajous curve at a given time.
     * 
     * @param time The time of the position.
     * @returns The position. 
     */
    inline Vector3d get_position(double time) override;

private:

    /**
     * @brief Initialise a new lissajous trajectory.
     * @param configuration The lissajous trajectory configuration.
     */
    inline LissajousTrajectory(const Configuration &configuration);

    /// The configuration of the lissajous trajectory.
    Configuration m_configuration;
};

struct PositionTrajectory::Configuration {

    enum Type {
        POINT,
        CIRCLE,
        RECTANGLE,
        LISSAJOUS
    };

    /// The configured position trajectory.
    Type type;

    /// Configuration for the point
    std::optional<PointTrajectory::Configuration> point;

    /// Configuration for the circle trajectory.
    std::optional<CircularTrajectory::Configuration> circle;

    /// Configuration for the rectangle trajectory.
    std::optional<RectangularTrajectory::Configuration> rectangle;

    /// Configuration for the lissajous trajectory.
    std::optional<LissajousTrajectory::Configuration> lissajous;
};
