#include "frankaridgeback/actor.hpp"

std::shared_ptr<FrankaRidgebackActor> FrankaRidgebackActor::create(
    Configuration &&configuration,
    Simulator &simulator
) {
    using namespace controller;

    auto controller = Controller::create(std::move(configuration.controller));
    if (!controller) {
        std::cerr << "failed to create FrankaRidgebackActor controller" << std::endl;
        return nullptr;
    }

    // The rate of updating 
    std:int64_t controller_countdown_max = (std::int64_t)(
        configuration.controller_rate / simulator.get_time_step()
    );

    // There must be at least one controller update.
    if (configuration.controller_substeps < 1) {
        configuration.controller_substeps = 1;
    }

    auto robot = simulator.get_world().addArticulatedSystem(configuration.urdf_filename);
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

    // Eigen::VectorXd pgain, dgain;
    // robot->getPdGains(pgain, dgain);
    // std::cout << "set position gain to " << pgain.transpose() << std::endl;
    // std::cout << "set velocity gain to " << dgain.transpose() << std::endl;

    // Start with zero force on the joints.
    auto zero = Eigen::VectorXd::Zero((Eigen::Index)robot->getDOF());
    robot->setGeneralizedForce(zero);

    return std::shared_ptr<FrankaRidgebackActor>(
        new FrankaRidgebackActor(
            std::move(configuration),
            std::move(controller),
            robot,
            controller_countdown_max
        )
    );
}

FrankaRidgebackActor::FrankaRidgebackActor(
    Configuration &&configuration,
    std::unique_ptr<controller::Controller> &&controller,
    raisim::ArticulatedSystem *robot,
    std::int64_t controller_countdown_max
) : m_configuration(std::move(configuration))
  , m_robot(robot)
  , m_state(FrankaRidgeback::State::Zero())
  , m_controller(std::move(controller))
  , m_controller_countdown(0) // Update on first step.
  , m_controller_countdown_max(controller_countdown_max)
{
    m_robot->setState(
        m_configuration.initial_state.position(),
        m_configuration.initial_state.velocity()
    );
}

inline FrankaRidgeback::State FrankaRidgebackActor::state()
{
    Eigen::VectorXd state_position, state_velocity;
    m_robot->getState(state_position, state_velocity);

    FrankaRidgeback::State state;
    state.position() = state_position;
    state.velocity() = state_velocity;

    state.end_effector_force() = m_robot->getExternalForce()[
        m_robot->getFrameIdxByName(m_configuration.end_effector_frame)
    ].e();

    state.end_effector_torque() = m_robot->getExternalTorque()[
        m_robot->getFrameIdxByName(m_configuration.end_effector_frame)
    ].e();

    return state;
}

void FrankaRidgebackActor::act(Simulator *simulator)
{
    using namespace FrankaRidgeback;
 
    if (--m_controller_countdown <= 0) {
        m_controller_countdown = m_controller_countdown_max;

        m_state = state();
        for (int i = 0; i < m_controller_substeps; i++)
            m_controller->update(m_state, simulator->get_time());
    }

    m_controller->get(m_control, simulator->get_time());

    // m_control.base_velocity() << 1.0, 0.0;
    // m_control.base_angular_velocity() << 1.0;
    // m_control.gripper_position() << 0.0, 0.0;
    // m_control.arm_torque() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Gripper positional controls.
    auto desired_position = Eigen::Vector<double, DoF::JOINTS>::Zero().eval();
    desired_position.tail<DoF::GRIPPER>() = m_control.gripper_position();

    // Base velocity controls.
    auto desired_velocity = Eigen::Vector<double, DoF::JOINTS>::Zero().eval();
    desired_velocity.head<DoF::BASE_POSITION>() = Eigen::Rotation2Dd(m_state.base_yaw().value()) * m_control.base_velocity();
    desired_velocity.segment<DoF::BASE_YAW>(DoF::BASE_POSITION) = m_control.base_angular_velocity();

    // Arm torque controls, add to the inherent gravitational and coriolis
    // forces on the joints.
    auto forces = m_robot->getNonlinearities(simulator->get_world().getGravity()).e().eval();
    forces.segment<DoF::ARM>(DoF::BASE) += m_control.arm_torque();

    // Set the PD controller targets and apply the torques.
    m_robot->setPdTarget(desired_position, desired_velocity);
    m_robot->setGeneralizedForce(forces);
}
