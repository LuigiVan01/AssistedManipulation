#include "controller/trajectory.hpp"

std::unique_ptr<PositionTrajectory> PositionTrajectory::create(
    const Configuration &configuration
) {
    std::unique_ptr<PositionTrajectory> trajectory = nullptr;

    // Todo: Something better than this.

    if (configuration.type == Configuration::Type::POINT) {
        if (!configuration.point) {
            std::cerr << "point trajectory selected without point configuration" << std::endl;
            return nullptr;
        }
        trajectory = PointTrajectory::create(*configuration.point); 
    }
    else if (configuration.type == Configuration::Type::CIRCLE) {
        if (!configuration.circle) {
            std::cerr << "circle trajectory selected without circle configuration" << std::endl;
            return nullptr;
        }
        trajectory = CircularTrajectory::create(*configuration.circle); 
    }
    else if (configuration.type == Configuration::Type::RECTANGLE) {
        if (!configuration.rectangle) {
            std::cerr << "rectangle trajectory selected without rectangle configuration" << std::endl;
            return nullptr;
        }
        trajectory = RectangularTrajectory::create(*configuration.rectangle); 
    }
    else if (configuration.type == Configuration::Type::LISSAJOUS) {
        if (!configuration.lissajous) {
            std::cerr << "lissajous trajectory selected without lissajous configuration" << std::endl;
            return nullptr;
        }
        trajectory = LissajousTrajectory::create(*configuration.lissajous); 
    }
    else {
        std::cerr << "unknown position trajectory type" << std::endl;
        return nullptr;
    }

    if (!trajectory) {
        std::cerr << "failed to create position trajectory" << std::endl;
        return nullptr;
    }

    return trajectory;
};

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
        std::cout << "cannot have zero velocity" << std::endl;
        return nullptr;
    }

    return std::unique_ptr<RectangularTrajectory>(
        new RectangularTrajectory(configuration)
    );
}

RectangularTrajectory::RectangularTrajectory(const Configuration &configuration)
    : m_configuration(configuration)
    , m_circumference(2 * configuration.width + 2 * configuration.height)
{
    m_transform.setIdentity();

    // Rotation from xy plane to the axis plane. No cartesian space warping.
    m_transform.linear() = Eigen::Quaterniond::FromTwoVectors(
        Vector3d(0, 0, 1),
        m_configuration.axis.normalized()
    ).normalized().toRotationMatrix();

    // Offset by the origin.
    m_transform.translation() = m_configuration.origin;
}

Vector3d RectangularTrajectory::get_position(double time)
{
    // Distance relative to the bottom left hand corner 
    double distance = std::remainder(
        time * m_configuration.velocity, m_circumference
    );

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
    double x = m_configuration.x_amplitude * std::sin(time);

    double y = m_configuration.y_amplitude * std::sin(
        m_configuration.y_frequency * time + m_configuration.y_phase
    );

    double z = m_configuration.z_amplitude * std::sin(
        m_configuration.z_frequency * time + m_configuration.z_phase
    );

    return m_configuration.origin + Vector3d(x, y, z);
}

/**
 * @brief Create a new static axis angle trajectory.
 * 
 * @param configuration The configuration of the axis angle trajectory.
 * @returns A pointer to the axis angle trajectory.
 */
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
