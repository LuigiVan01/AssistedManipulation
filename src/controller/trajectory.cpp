#include "controller/trajectory.hpp"

#include <cmath>

template<typename T, typename Configuration>
std::unique_ptr<T> try_configuration(
    const std::optional<Configuration> &configuration,
    std::string name
) {
    if (!configuration) {
        std::cerr << name << "trajectory selected without "
                    << name << "configuration" << std::endl;
        return std::unique_ptr<T>();
    }

    auto trajectory = T::create(*configuration);
    if (!trajectory)
        std::cerr << "failed to create " << name << " trajectory" << std::endl;

    return trajectory;
}

std::unique_ptr<PositionTrajectory> PositionTrajectory::create(
    const Configuration &configuration
) {
    switch (configuration.type)
    {
        case Configuration::Type::POINT: {
            return try_configuration<PointTrajectory, PointTrajectory::Configuration>(
                configuration.point,
                "point"
            );
        }
        case Configuration::Type::CIRCLE: {
            return try_configuration<CircularTrajectory, CircularTrajectory::Configuration>(
                configuration.circle,
                "circular"
            );
        }
        case Configuration::Type::RECTANGLE: {
            return try_configuration<RectangularTrajectory, RectangularTrajectory::Configuration>(
                configuration.rectangle,
                "rectangular"
            );
        }
        case Configuration::Type::LISSAJOUS: {
            return try_configuration<LissajousTrajectory, LissajousTrajectory::Configuration>(
                configuration.lissajous,
                "lissajous"
            );
        }
        case Configuration::Type::FIGURE_EIGHT: {
            return try_configuration<FigureEightTrajectory, FigureEightTrajectory::Configuration>(
                configuration.figure_eight,
                "figure eight"
            );
        }
        default: {
            std::cerr << "unknown position trajectory type"
                      << configuration.type << std::endl;
            return nullptr;
        }
    }
};

std::unique_ptr<OrientationTrajectory> OrientationTrajectory::create(
    const Configuration &configuration
) {
    switch (configuration.type)
    {
        case Configuration::Type::AXIS_ANGLE: {
            return try_configuration<AxisAngleTrajectory, AxisAngleTrajectory::Configuration>(
                configuration.axis_angle,
                "axis angle"
            );
            break;
        }
        case Configuration::Type::SLERP: {
            return try_configuration<SlerpTrajectory, SlerpTrajectory::Configuration>(
                configuration.slerp,
                "slerp"
            );
            break;
        }
        default: {
            std::cerr << "unknown orientation trajectory type"
                      << configuration.type << std::endl;
            return nullptr;
        }
    }
}

std::unique_ptr<PointTrajectory> PointTrajectory::create(
    const Configuration &configuration
) {
    return std::unique_ptr<PointTrajectory>(
        new PointTrajectory(configuration)
    );
}

PointTrajectory::PointTrajectory(const Configuration &configuration)
    : m_point(configuration.point)
{}

Vector3d PointTrajectory::get_position(double /* time */)
{
    return m_point;
}

std::unique_ptr<CircularTrajectory> CircularTrajectory::create(
    const Configuration &configuration
) {
    return std::unique_ptr<CircularTrajectory>(
        new CircularTrajectory(configuration)
    );
}

CircularTrajectory::CircularTrajectory(const Configuration &configuration)
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

Vector3d CircularTrajectory::get_position(double time)
{
    return m_origin + Eigen::AngleAxisd(
        time * m_angular_velocity,
        m_axis
    ) * m_point;
}

std::unique_ptr<RectangularTrajectory> RectangularTrajectory::create(
    const Configuration &configuration
) {
    if (configuration.velocity < 0) {
        std::cerr << "cannot have non-positive velocity" << std::endl;
        return nullptr;
    }

    return std::unique_ptr<RectangularTrajectory>(
        new RectangularTrajectory(configuration)
    );
}

RectangularTrajectory::RectangularTrajectory(const Configuration &configuration)
    : m_configuration(configuration)
    , m_circumference(2 * configuration.width + 2 * configuration.height)
    , m_velocity(m_configuration.velocity)
{
    m_transform.setIdentity();

    // Rotation from xy plane to the axis plane. No cartesian space warping.
    m_transform.linear() = Eigen::Quaterniond::FromTwoVectors(
        Vector3d(0, 0, 1),
        m_configuration.axis.normalized()
    ).normalized().toRotationMatrix();

    // Offset by the origin, and translate the bottom left corner to the
    // origin.
    m_transform.translation() = m_configuration.origin - Vector3d(
        configuration.width / 2, configuration.height / 2, 0
    );
}

Vector3d RectangularTrajectory::get_position(double time)
{
    // Distance relative to the bottom left hand corner 
    double distance = std::fmod(time * m_velocity, m_circumference);

    if (distance < m_configuration.width) {
        return m_transform * Vector3d(distance, 0, 0.0);
    }

    distance -= m_configuration.width;

    if (distance < m_configuration.height) {
        return m_transform * Vector3d(m_configuration.width, distance, 0.0);
    }

    distance -= m_configuration.height;

    if (distance < m_configuration.width) {
        return m_transform * Vector3d(
            m_configuration.width - distance,
            m_configuration.height,
            0.0
        );
    }

    distance -= m_configuration.width;

    return m_transform * Vector3d(0, m_configuration.height - distance, 0.0);
}

std::unique_ptr<LissajousTrajectory> LissajousTrajectory::create(
    const Configuration &configuration
) {
    return std::unique_ptr<LissajousTrajectory>(
        new LissajousTrajectory(configuration)
    );
}

LissajousTrajectory::LissajousTrajectory(const Configuration &configuration)
    : m_configuration(configuration)
{}

Vector3d LissajousTrajectory::get_position(double time)
{
    Vector3d position = m_configuration.origin + Vector3d(
        m_configuration.x_amplitude * std::sin(
            m_configuration.x_frequency * time
        ),
        m_configuration.y_amplitude * std::sin(
            m_configuration.y_frequency * time + m_configuration.y_phase
        ),
        m_configuration.z_amplitude * std::sin(
            m_configuration.z_frequency * time + m_configuration.z_phase
        )
    );
    return position;
}

std::unique_ptr<FigureEightTrajectory> FigureEightTrajectory::create(
    const Configuration &configuration
) {
    // Figure 8 trajectory along the x axis.
    LissajousTrajectory::Configuration lissajous {
        .origin = configuration.origin,
        .x_amplitude = configuration.x_amplitude,
        .y_amplitude = configuration.y_amplitude,
        .z_amplitude = 0.0,
        .x_frequency = configuration.frequency,
        .y_frequency = 2 * configuration.frequency,
        .z_frequency = 0.0,
        .y_phase = M_PI,
        .z_phase = 0.0
    };

    return std::unique_ptr<FigureEightTrajectory>(
        new FigureEightTrajectory(lissajous)
    );
}

std::unique_ptr<AxisAngleTrajectory> AxisAngleTrajectory::create(
    const Configuration &configuration
) {
    return std::unique_ptr<AxisAngleTrajectory>(
        new AxisAngleTrajectory(configuration)
    );
}

AxisAngleTrajectory::AxisAngleTrajectory(const Configuration &configuration)
    : m_orientation(Quaterniond::Identity())
{
    // Rotate by the angle around the z axis, the rotate the body from the z
    // axis to the provided axis.
    m_orientation = (
        Eigen::AngleAxisd(configuration.angle, Vector3d(0, 0, 1)) *
        Quaterniond::FromTwoVectors(Vector3d(0, 0, 1), configuration.axis)
    );
}

Quaterniond AxisAngleTrajectory::get_orientation(double /* time */)
{
    return m_orientation;
}

std::unique_ptr<SlerpTrajectory> SlerpTrajectory::create(
    const Configuration &configuration
) {
    return std::unique_ptr<SlerpTrajectory>(
        new SlerpTrajectory(configuration)
    );
}

SlerpTrajectory::SlerpTrajectory(
    const Configuration &configuration
) : m_first(Quaterniond::Identity())
  , m_second(Quaterniond::Identity())
  , m_frequency(configuration.frequency)
{
    m_first = (
        Eigen::AngleAxisd(configuration.first_angle, Vector3d(0, 0, 1)) *
        Quaterniond::FromTwoVectors(Vector3d(0, 0, 1), configuration.first_axis)
    );

    m_second = (
        Eigen::AngleAxisd(configuration.second_angle, Vector3d(0, 0, 1)) *
        Quaterniond::FromTwoVectors(Vector3d(0, 0, 1), configuration.second_axis)
    );
}

Quaterniond SlerpTrajectory::get_orientation(double time)
{
    // Normalise the sine wave between zero and one.
    double t = (std::sin(time) + 1.0) / 2.0; 

    // Spherically interpolate between the two orientations.
    return m_first.slerp(t, m_second);
}
