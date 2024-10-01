#include "test/case/trajectory.hpp"

const TrajectoryTest::Configuration TrajectoryTest::DEFAULT_CONFIGURATION {
    .duration = 300.0,
    .position = PositionTrajectory::Configuration {
        .type = PositionTrajectory::Configuration::Type::LISSAJOUS,
        .point = PointTrajectory::DEFAULT_CONFIGURATION,
        .circle = CircularTrajectory::DEFAULT_CONFIGURATION,
        .rectangle = RectangularTrajectory::DEFAULT_CONFIGURATION,
        .lissajous = LissajousTrajectory::DEFAULT_CONFIGURATION,
        .figure_eight = FigureEightTrajectory::DEFAULT_CONFIGURATION
    },
    // .orientation = std::nullopt,
    .orientation = OrientationTrajectory::Configuration {
        .type = OrientationTrajectory::Configuration::Type::SLERP,
        .axis_angle = AxisAngleTrajectory::DEFAULT_CONFIGURATION,
        .slerp = SlerpTrajectory::DEFAULT_CONFIGURATION
    }
};

std::unique_ptr<TrajectoryTest> TrajectoryTest::create(Options &options)
{
    Configuration configuration = DEFAULT_CONFIGURATION;
    configuration.duration = options.duration;

    // If configuration overrides were provided, apply them based on the json
    // patch specification.
    try {
        if (!options.patch.is_null()) {
            json json_configuration = DEFAULT_CONFIGURATION;
            json_configuration.merge_patch(options.patch);
            configuration = json_configuration;
        }
    }
    catch (const json::exception &err) {
        std::cerr << "error when patching json configuration: " << err.what() << std::endl;
        std::cerr << "configuration was " << ((json)DEFAULT_CONFIGURATION).dump(4) << std::endl;
        std::cerr << "patch was " << options.patch.dump(4) << std::endl;
        return nullptr;
    }

    return create(configuration);
}

std::unique_ptr<TrajectoryTest> TrajectoryTest::create(
    const Configuration &configuration
) {
    auto simulator = Simulator::create(Simulator::DEFAULT_CONFIGURATION);
    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    if (!configuration.position && !configuration.orientation) {
        std::cerr << "at least one position or orientation trajectory is required" << std::endl;
        return nullptr;
    }

    Vector3d origin = Vector3d::Zero();

    // Create the positional part of the trajectory if provided.
    std::unique_ptr<PositionTrajectory> position;
    if (configuration.position) {
        position = PositionTrajectory::create(*configuration.position);
        if (!position) {
            std::cerr << "failed to create position trajectory" << std::endl;
            return nullptr;
        }

        switch (configuration.position->type)
        {
            case PositionTrajectory::Configuration::POINT: {
                origin = configuration.position->point->point;
                break;
            }
            case PositionTrajectory::Configuration::CIRCLE: {
                origin = configuration.position->circle->origin;
                break;
            }
            case PositionTrajectory::Configuration::RECTANGLE: {
                origin = configuration.position->rectangle->origin;
                break;
            }
            case PositionTrajectory::Configuration::LISSAJOUS: {
                origin = configuration.position->lissajous->origin;
                break;
            }
            case PositionTrajectory::Configuration::FIGURE_EIGHT: {
                origin = configuration.position->figure_eight->origin;
                break;
            }
        }
    }

    // Create the orientation part of the trajectory if provided.
    std::unique_ptr<OrientationTrajectory> orientation;
    if (configuration.orientation) {
        orientation = OrientationTrajectory::create(*configuration.orientation);
        if (!orientation) {
            std::cerr << "failed to create orientation trajectory" << std::endl;
            return nullptr;
        }
    }

    return std::unique_ptr<TrajectoryTest>(
        new TrajectoryTest(
            configuration,
            std::move(simulator),
            std::move(position),
            std::move(orientation),
            origin
        )
    );
}

TrajectoryTest::TrajectoryTest(
    const Configuration &configuration,
    std::unique_ptr<Simulator> &&simulator,
    std::unique_ptr<PositionTrajectory> &&position,
    std::unique_ptr<OrientationTrajectory> &&orientation,
    Vector3d origin
 ) : m_duration(configuration.duration)
   , m_simulator(std::move(simulator))
   , m_position(std::move(position))
   , m_orientation(std::move(orientation))
{
    m_sphere = m_simulator->get_server().addVisualSphere(
        "trajectory_position",
        0.05
    );

    m_arrow = m_simulator->get_server().addVisualArrow(
        "trajectory_orientation",
        0.2, 0.2
    );

    // Bug, changing the origin changes the camera orientation.
    m_simulator->get_server().setCameraPositionAndLookAt(
        origin + Vector3d(0, 0.5, 0.5),
        origin
    );
}

bool TrajectoryTest::run()
{
    double time = m_simulator->get_time();

    auto position = Vector3d(0, 0, 1);
    auto orientation = Quaterniond::Identity();

    while (time < m_duration) {
        auto delay = raisim::TimedLoop(m_simulator->get_time_step() * 1e6);

        if (m_position)
            position = m_position->get_position(time);

        if (m_orientation)
            orientation = m_orientation->get_orientation(time);

        m_sphere->setPosition(position);
        m_arrow->setPosition(position);
        m_arrow->setOrientation(orientation.coeffs());

        m_simulator->step();
        time = m_simulator->get_time();
    }

    return true;
}
