#include "test/case/pinocchio.hpp"

const static PinocchioDynamicsTest::Configuration DEFAULT_CONFIGURATION {
    .simulator = {
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    },
    .dynamics = {
        .filename = "",
        .end_effector_frame = "panda_grasp",
        .energy = 10.0
    },
    .force = {
        .observation = Eigen::Vector<double, 6>(0, 0, 0, 0, 0, 0)
    },
    .initial_state = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        10.0
    }
};

std::unique_ptr<Test> PinocchioDynamicsTest::create(Options &options)
{
    PinocchioDynamicsTest::Configuration configuration = DEFAULT_CONFIGURATION;
    configuration.duration = options.duration;
    return create(configuration);
}

std::unique_ptr<Test> PinocchioDynamicsTest::create(const Configuration &configuration)
{
    auto simulator = Simulator::create(configuration.simulator);
    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    auto force = LOCFForecast::create(configuration.force);
    if (!force) {
        std::cerr << "failed to create locf forecast" << std::endl;
        return nullptr;
    }

    FrankaRidgeback::Dynamics::Configuration dynamics_configuration = configuration.dynamics;
    if (dynamics_configuration.filename.empty())
        dynamics_configuration.filename = FrankaRidgeback::Dynamics::find_path().string();

    auto dynamics = FrankaRidgeback::Dynamics::create(dynamics_configuration, force.get());
    if (!dynamics) {
        std::cerr << "failed to create dynamics" << std::endl;
        return nullptr;
    }

    dynamics->set(configuration.initial_state, 0.0);

    auto visual = simulator->get_server().addVisualArticulatedSystem(
        "pinocchio",
        dynamics_configuration.filename
    );
    if (!visual) {
        std::cerr << "failed to create pinocchio visualiser" << std::endl;
        return nullptr;
    }

    return std::unique_ptr<PinocchioDynamicsTest>(
        new PinocchioDynamicsTest(
            std::move(simulator),
            std::move(dynamics),
            std::move(force),
            visual,
            configuration.duration
        )
    );
}

PinocchioDynamicsTest::PinocchioDynamicsTest(
        std::unique_ptr<Simulator> &&simulator,
        std::unique_ptr<FrankaRidgeback::Dynamics> &&dynamics,
        std::unique_ptr<LOCFForecast> &&force,
        raisim::ArticulatedSystemVisual *visual,
        double duration
  ) : m_duration(duration)
    , m_simulator(std::move(simulator))
    , m_dynamics(std::move(dynamics))
    , m_force(std::move(force))
    , m_visual(visual)
{}

bool PinocchioDynamicsTest::run()
{
    double time = 0.0;
    auto zero = FrankaRidgeback::Control::Zero();

    while (time < m_duration) {
        raisim::TimedLoop(m_simulator->get_world().getTimeStep() * 1e6);
        FrankaRidgeback::State state = m_dynamics->step(zero, m_simulator->get_time_step());
        m_visual->setGeneralizedCoordinate(state.position());
        m_simulator->step();
        time += m_simulator->get_time_step();
    }

    return true;
}
