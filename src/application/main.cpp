// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
#include <cstdlib>
#include <filesystem>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "controller/controller.hpp"

int main(int argc, char* argv[])
{
    auto cwd = std::filesystem::current_path();

    raisim::World::setActivationKey(std::getenv("RAISIM_ACTIVATION"));
    std::cout << argv[0] << std::endl;

    raisim::World world;
    world.setTimeStep(0.005);

    /// create objects
    auto ground = world.addGround();
    ground->setName("ground");
    ground->setAppearance("grid");

    auto panda = world.addArticulatedSystem((cwd / "robot.urdf").string());

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();

    while (1) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();
    }

    server.killServer();
}
