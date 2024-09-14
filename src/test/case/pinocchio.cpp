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
        .observation = Eigen::Vector3d(0, 0, 1);
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
    if (dynamics_configuration.model.filename.empty())
        dynamics_configuration.model.filename = FrankaRidgeback::Dynamics::find_path().string();

    auto dynamics = FrankaRidgeback::Dynamics::create(configuration.dynamics, force.get());
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

    auto test = std::unique_ptr<PinocchioDynamicsTest>(new PinocchioDynamicsTest());
    test->m_simulator = std::move(simulator);
    test->m_dynamics = std::move(dynamics);
    test->m_visual = std::move(visual);
    return test;
}

bool PinocchioDynamicsTest::run()
{
    double time = 0.0;
    while (time < time.) {
        
        m_visual->setGeneralizedCoordinate();
    }
}
