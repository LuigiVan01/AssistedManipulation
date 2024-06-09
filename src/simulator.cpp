#include "simulator.hpp"

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

namespace simulator {

std::unique_ptr<Simulator> Simulator::create(std::string urdf_filename)
{
    raisim::World::setActivationKey(std::getenv("RAISIM_ACTIVATION"));

    raisim::World world;
    world.setTimeStep(0.005);

    auto ground = world.addGround();
    ground->setName("ground");
    ground->setAppearance("grid");

    auto robot = world.addArticulatedSystem(urdf_filename);

    auto zero = Eigen::VectorXd::Zero((Eigen::Index)robot->getDOF());

    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    robot->setPdGains(zero, zero);
    robot->setGeneralizedForce(zero);

    return std::make_unique<Simulator>(
        std::move(world),
        std::move(robot)
    );
}

Simulator::Simulator(raisim::World &&world, raisim::ArticulatedSystem *robot)
    : m_world(std::move(world))
    , m_robot(robot)
    , m_server(&world)
{
    m_server.launchServer();
}

void Simulator::step(FrankaRidgeback::Control control, double dt)
{
    // Position controls.
    m_position_control.head<FrankaRidgeback::DoF::ARM>().setZero();
    m_position_control.tail<FrankaRidgeback::DoF::GRIPPER>() = m_state.gripper_position();

    // Velocity controls.
    m_velocity_control.head<FrankaRidgeback::DoF::ARM>() = control.arm_torque();
    m_velocity_control.tail<FrankaRidgeback::DoF::GRIPPER>().setZero();

    // m_robot->setGeneralizedForce();
    m_robot->setPdTarget(m_position_control, m_velocity_control);
}

} // namespace simulator