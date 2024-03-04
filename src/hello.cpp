#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include <Eigen/Core>

int main() {
  raisim::World::setActivationKey("activation.raisim");
  raisim::World world;
  // auto anymal = world.addArticulatedSystem(PATH_TO_URDF);
  auto ball = world.addSphere(1, 1);
  auto ground = world.addGround();
  world.setTimeStep(0.002);

  /// launch raisim server for visualization. Can be visualized on raisimUnity
  raisim::RaisimServer server(&world);
  server.launchServer();

  while (1) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
