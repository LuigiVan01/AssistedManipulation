#include "simulator.hpp"

std::unique_ptr<Simulator> Simulator::create(const Configuration &configuration)
{
    raisim::World::setActivationKey(std::getenv("RAISIM_ACTIVATION"));

    auto world = std::make_unique<raisim::World>();
    world->setTimeStep(configuration.timestep);

    auto ground = world->addGround();
    ground->setName("ground");
    ground->setAppearance("grid");

    auto robot = world->addArticulatedSystem(configuration.urdf_filename);
    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);

    auto zero = Eigen::VectorXd::Zero((Eigen::Index)robot->getDOF());
    robot->setPdGains(zero, zero);
    robot->setGeneralizedForce(zero);

    return std::unique_ptr<Simulator>(
        new Simulator(configuration, std::move(world), robot)
    );
}

Simulator::Simulator(
    const Configuration &configuration,
    std::unique_ptr<raisim::World> &&world,
    raisim::ArticulatedSystem *robot
) : m_configuration(configuration)
  , m_time(0.0)
  , m_world(std::move(world))
  , m_robot(robot)
  , m_server(m_world.get())
{
    reset(m_configuration.initial_state);
    m_server.launchServer();
}

Simulator::~Simulator()
{
    m_server.killServer();
}

void Simulator::reset(FrankaRidgeback::State &state)
{
    m_position_control.setZero();
    m_velocity_control.setZero();
    m_robot->setState(state.joint_positions(), state.joint_velocities());
}

const FrankaRidgeback::State &Simulator::step(FrankaRidgeback::Control &control)
{
    // Position controls.
    m_position_control.head<FrankaRidgeback::DoF::ARM>().setZero();
    m_position_control.tail<FrankaRidgeback::DoF::GRIPPER>() = m_state.gripper_position();

    // Velocity controls.
    m_velocity_control.head<FrankaRidgeback::DoF::ARM>() = control.arm_torque();
    m_velocity_control.tail<FrankaRidgeback::DoF::GRIPPER>().setZero();

    // Set the raisim controller to target position and velocity.
    m_robot->setPdTarget(m_position_control, m_velocity_control);
    // m_robot->setGeneralizedForce(m_robot->getNonlinearities(m_configuration.gravity));

    // Simulate!
    m_server.integrateWorldThreadSafe();
    m_time += m_configuration.timestep;

    Eigen::VectorXd position, velocity;
    m_robot->getState(position, velocity);

    m_state.joint_positions() = position;
    m_state.joint_velocities() = velocity;

    return m_state;
}
