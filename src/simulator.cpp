#include "simulator.hpp"

std::unique_ptr<Simulator> Simulator::create(const Configuration &configuration)
{
    raisim::World::setActivationKey(std::getenv("RAISIM_ACTIVATION"));

    auto world = std::make_unique<raisim::World>();
    world->setTimeStep(configuration.timestep);
    world->setGravity(configuration.gravity);

    auto ground = world->addGround();
    ground->setName("ground");
    ground->setAppearance("grid");

    auto robot = world->addArticulatedSystem(configuration.urdf_filename);
    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    std::cout << "simulated robot DoF = " << robot->getDOF() << std::endl;

    // The base and the gripper use proportional derivative control for their
    // joints, whereas the arm uses feed-forward torque commands. Therefore, the
    // PD gains of the torque commands should be zero (disable PD control) and
    // the PD gains of the base and the gripper should be one (enable PD
    // control). This is true for both position and velocity control.

    FrankaRidgeback::Control position_gain = FrankaRidgeback::Control::Zero();
    position_gain.base().setOnes();
    position_gain.arm_torque().setZero();
    position_gain.gripper_position().setOnes();

    FrankaRidgeback::Control velocity_gain = FrankaRidgeback::Control::Zero();
    velocity_gain.base().setOnes();
    velocity_gain.arm_torque().setZero();
    velocity_gain.gripper_position().setOnes();

    robot->setPdGains(position_gain, velocity_gain);

    // Start with zero force on the joints.
    auto zero = Eigen::VectorXd::Zero((Eigen::Index)robot->getDOF());
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
  , m_state(FrankaRidgeback::State::Zero())
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
    m_robot->setState(state.position(), state.velocity());
}

const FrankaRidgeback::State &Simulator::step(FrankaRidgeback::Control &control)
{
    using namespace FrankaRidgeback;

    // control.base_velocity() << 0.0, 0.0;
    // control.base_rotation() << 0.0;
    // control.gripper_position() << 0.0, 0.0;
    // control.arm_torque() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963267948966;

    // Rotate the velocity vector to the direction of the base.
    double rotation = control.base_rotation().coeff(0);
    control.base_velocity() = Eigen::Rotation2Dd(rotation) * control.base_velocity();

    // Positional controls.
    auto desired_position = Eigen::Vector<double, DoF::JOINTS>::Zero().eval();
    desired_position.segment<DoF::BASE_ROTATION>(DoF::BASE_VELOCITY) = control.base_rotation();
    desired_position.tail<DoF::GRIPPER>() = control.gripper_position();

    // Velocity controls.
    auto desired_velocity = Eigen::Vector<double, DoF::JOINTS>::Zero().eval();
    desired_velocity.head<DoF::BASE_VELOCITY>() = control.base_velocity();

    // Set the raisim controller to target position and velocity.
    m_robot->setPdTarget(desired_position, desired_velocity);

    // Add the desired arm torque to the coriolis and gravity forces.
    auto torque = m_robot->getNonlinearities(m_world->getGravity()).e().eval();
    torque.segment<DoF::ARM>(DoF::BASE) = control.arm_torque();
    m_robot->setGeneralizedForce(torque);

    // Simulate!
    m_server.integrateWorldThreadSafe();
    m_time += m_configuration.timestep;

    // Get the next state.
    Eigen::VectorXd position, velocity;
    m_robot->getState(position, velocity);

    m_state.arm_position() = position.head<DoF::ARM>();
    m_state.gripper_position() = position.tail<DoF::GRIPPER>();
    m_state.arm_velocity() = velocity.head<DoF::ARM>();
    m_state.gripper_velocity() = velocity.tail<DoF::GRIPPER>();

    return m_state;
}
