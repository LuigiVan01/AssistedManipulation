// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
#include <cstdlib>
#include <filesystem>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "cost.hpp"
#include "dynamics.hpp"
#include "mppi.hpp"

int main(int /* argc */, char*[])
{
    auto cwd = std::filesystem::current_path();

    raisim::World::setActivationKey(std::getenv("RAISIM_ACTIVATION"));

    raisim::World world;
    world.setTimeStep(0.005);

    /// create objects
    auto ground = world.addGround();
    ground->setName("ground");
    ground->setAppearance("grid");

    std::string urdf = (cwd / "model/robot.urdf").string();
    auto panda = world.addArticulatedSystem(urdf);

    // Create the controller.
    mppi::Configuration configuration {
        .rollouts = 10,
        .rollouts_cached = 0,
        .step_size = 0.05,
        .horison = 1.0,
        .gradient_step = 1.0,
        .gradient_minmax = 10000.0,
        .cost_scale = 1.0,
        .cost_discount_factor = 0.9,
        .control_default_last = true,
        .control_default_value = FrankaRidgeback::Dynamics::Control::Zero(),
    };

    // Set the initial state.
    auto initial_state = FrankaRidgeback::State::Zero();

    std::cout << "creating dynamics" << std::endl;
    auto dynamics = FrankaRidgeback::Dynamics::create();
    if (!dynamics) {
        std::cerr << "failed to create dynamics" << std::endl;
        return 1;
    }

    std::cout << "creating cost" << std::endl;
    auto cost = Cost::create(urdf);
    if (!cost) {
        std::cerr << "failed to create cost" << std::endl;
        return 1;
    }

    std::cout << "creating trajectory" << std::endl;
    auto trajectory = mppi::Trajectory<FrankaRidgeback::Dynamics, Cost>::create(
        dynamics,
        cost,
        initial_state,
        configuration
    );

    if (!trajectory) {
        std::cerr << "failed to create trajectory" << std::endl;
        return 1;
    }

    panda->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    panda->setPdGains(FrankaRidgeback::Control::Zero(), FrankaRidgeback::Control::Zero());
    panda->setGeneralizedForce(Eigen::VectorXd::Zero((Eigen::Index)panda->getDOF()));

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();

    double mppi_frequency = 0.5;
    std::size_t updates = mppi_frequency / world.getTimeStep();

    for (;;) {
        // trajectory->update();

        for (std::size_t i = 0; i < updates; i++) {

            // Set the trajectory.

            auto delay = raisim::TimedLoop((std::size_t)(world.getTimeStep() * 1e6));
            server.integrateWorldThreadSafe();
        }
    }

    server.killServer();
}
