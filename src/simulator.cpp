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

    robot->setPdGains(
        configuration.proportional_gain,
        configuration.differential_gain
    );

    Eigen::VectorXd pgain, dgain;
    robot->getPdGains(pgain, dgain);
    std::cout << "set position gain to " << pgain.transpose() << std::endl;
    std::cout << "set velocity gain to " << dgain.transpose() << std::endl;

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

    auto sphere = m_server.addVisualSphere("sphere", 0.1);
    sphere->setPosition(1.0, 1.0, 1.0);
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

    // Gripper positional controls.
    auto desired_position = Eigen::Vector<double, DoF::JOINTS>::Zero().eval();
    desired_position.tail<DoF::GRIPPER>() = control.gripper_position();

    // Base velocity controls.
    auto desired_velocity = Eigen::Vector<double, DoF::JOINTS>::Zero().eval();
    desired_velocity.head<DoF::BASE_POSITION>() = Eigen::Rotation2Dd(m_state.base_yaw().value()) * control.base_velocity();
    desired_velocity.segment<DoF::BASE_YAW>(DoF::BASE_POSITION) = control.base_angular_velocity();

    // Arm torque controls, add to the inherent gravitational and coriolis
    // forces on the joints.
    auto forces = m_robot->getNonlinearities(m_world->getGravity()).e().eval();
    forces.segment<DoF::ARM>(DoF::BASE) += control.arm_torque();

    // Set the PD controller targets and apply the torques.
    m_robot->setPdTarget(desired_position, desired_velocity);
    m_robot->setGeneralizedForce(forces);

    // Once the robot joint control parameters for this time step have been set,
    // simulate!
    m_server.integrateWorldThreadSafe();
    m_time += m_configuration.timestep;

    // Get the next state.
    Eigen::VectorXd state_position, state_velocity;
    m_robot->getState(state_position, state_velocity);
    m_state.position() = state_position;
    m_state.velocity() = state_velocity;

    return m_state;
}
