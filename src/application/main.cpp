// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
#include <cstdlib>
#include <filesystem>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "controller.hpp"
#include "application/cost.hpp"
#include "application/dynamics.hpp"

int main(int /* argc */, char* argv[])
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

    controller::Configuration configuration {
        .rollouts = 10,
        .rollouts_cached = 0,
        .step_size = 0.05,
        .horison = 1.0,
        .gradient_step = 1.0,
        .gradient_minmax = 10000.0,
        .cost_scale = 1.0,
        .cost_discount_factor = 0.9,
        .control_default_last = true,
        .control_default_value = Dynamics::Control::Zero(),
    };

    auto initial_state = Dynamics::State::Zero();

    auto dynamics = Dynamics::create();
    auto cost = Cost::create(urdf);

    auto trajectory = controller::Trajectory<Dynamics, Cost>::create(
        dynamics,
        cost,
        initial_state,
        configuration
    );

    panda->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    panda->setPdGains(Dynamics::Control::Zero(), Dynamics::Control::Zero());
    panda->setGeneralizedForce(Eigen::VectorXd::Zero((Eigen::Index)panda->getDOF()));

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();

    while (1) {
        RS_TIMED_LOOP((std::size_t)(world.getTimeStep() * 1e6))
        server.integrateWorldThreadSafe();
    }

    server.killServer();
}
