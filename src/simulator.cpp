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

    std::cout << "simulated robot of DoF " << robot->getDOF() << std::endl;
    int i = 0;
    for (auto name : robot->getMovableJointNames()) {
        auto joint = robot->getJoint(name);
        const char *type;

        switch (joint.getType()) {
            case raisim::Joint::FIXED: type = "fixed"; break;
            case raisim::Joint::REVOLUTE: type = "revolute"; break;
            case raisim::Joint::PRISMATIC: type = "prismatic"; break;
            case raisim::Joint::SPHERICAL: type = "spherical"; break;
            case raisim::Joint::FLOATING: type = "floating"; break;
        }

        std::cout << "    joint " << i << " (" << name << ") is " << type << std::endl;
        i++;
    }

    // The base and the gripper use proportional derivative control for their
    // joints, whereas the arm uses feed-forward torque commands. Therefore, the
    // PD gains of the torque commands should be zero (disable PD control) and
    // the PD gains of the base and the gripper should be one (enable PD
    // control). This is true for both position and velocity control.

    FrankaRidgeback::Control position_gain = FrankaRidgeback::Control::Zero();
    position_gain.base_velocity() << 0.0, 0.0;
    position_gain.base_angular_velocity() << 0.0;
    position_gain.arm_torque() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    position_gain.gripper_position() << 100.0, 100.0;

    FrankaRidgeback::Control velocity_gain = FrankaRidgeback::Control::Zero();
    velocity_gain.base_velocity() << 1000.0, 1000.0;
    velocity_gain.base_angular_velocity() << 1.0;
    velocity_gain.arm_torque() << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    velocity_gain.gripper_position() << 50.0, 50.0;

    robot->setPdGains(position_gain, velocity_gain);

    Eigen::VectorXd pgain, dgain;
    robot->getPdGains(pgain, dgain);
    std::cout << "set pd gains to " << pgain << " and " << dgain << std::endl;

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

    // control.base_velocity() << 1.0, 0.0;
    // control.base_angular_velocity() << 1.0;
    // control.gripper_position() << 0.0, 0.0;
    // control.arm_torque() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Positional controls.
    auto desired_position = Eigen::Vector<double, DoF::JOINTS>::Zero().eval();
    desired_position.tail<DoF::GRIPPER>() = control.gripper_position();

    // Velocity controls.
    auto desired_velocity = Eigen::Vector<double, DoF::JOINTS>::Zero().eval();
    desired_velocity.segment<DoF::BASE_ANGLE>(DoF::BASE_VELOCITY) = control.base_angular_velocity();
    desired_velocity.head<DoF::BASE_VELOCITY>() = Eigen::Rotation2Dd(m_state.base_angle().value()) * control.base_velocity();

    // Add torque to the arm joints.
    auto forces = m_robot->getNonlinearities(m_world->getGravity()).e().eval();
    forces.segment<DoF::ARM>(DoF::BASE) += control.arm_torque();

    m_robot->setPdTarget(desired_position, desired_velocity);
    m_robot->setGeneralizedForce(forces);

    // Simulate!
    m_server.integrateWorldThreadSafe();
    m_time += m_configuration.timestep;

    // Get the next state.
    Eigen::VectorXd state_position, state_velocity;
    m_robot->getState(state_position, state_velocity);
    m_state.position() = state_position;
    m_state.velocity() = state_velocity;

    return m_state;
}
