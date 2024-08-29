#include "simulation/actors/frankaridgeback.hpp"

std::shared_ptr<FrankaRidgebackActor> FrankaRidgebackActor::create(
    const Configuration &configuration,
    Simulator *simulator,
    std::unique_ptr<mppi::Dynamics> &&dynamics,
    std::unique_ptr<mppi::Cost> &&cost,
    std::unique_ptr<mppi::Filter> &&filter
) {
    using namespace controller;

    auto controller = mppi::Trajectory::create(
        configuration.mppi,
        std::move(dynamics),
        std::move(cost),
        std::move(filter)
    );

    if (!controller) {
        std::cerr << "failed to create FrankaRidgebackActor mppi" << std::endl;
        return nullptr;
    }

    // The rate of updating 
    std::int64_t controller_countdown_max = (std::int64_t)(
        configuration.controller_rate / simulator->get_time_step()
    );

    // There must be at least one controller update.
    if (configuration.controller_substeps < 1) {
        std::cerr << "frankaridgeback actor substeps less than zero" << std::endl;
        return nullptr;
    }

    auto robot = simulator->get_world().addArticulatedSystem(configuration.urdf_filename);
    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);

    std::size_t end_effector_frame_index = robot->getFrameIdxByName(
        configuration.end_effector_frame
    );

    if (end_effector_frame_index > robot->getFrames().size() - 1) {
        std::cerr << "end effector frame \""
                  << configuration.end_effector_frame
                  << "\" does not exist " << std::endl;
        std::cerr << "existing frames: ";
        for (auto &x : robot->getFrames())
            std::cerr << x.name << ", ";
        std::cerr << std::endl;
        return nullptr;
    }

    // std::cout << "simulated robot of DoF " << robot->getDOF() << std::endl;
    // int i = 0;
    // for (auto name : robot->getMovableJointNames()) {
    //     auto joint = robot->getJoint(name);
    //     const char *type;

    //     switch (joint.getType()) {
    //         case raisim::Joint::FIXED: type = "fixed"; break;
    //         case raisim::Joint::REVOLUTE: type = "revolute"; break;
    //         case raisim::Joint::PRISMATIC: type = "prismatic"; break;
    //         case raisim::Joint::SPHERICAL: type = "spherical"; break;
    //         case raisim::Joint::FLOATING: type = "floating"; break;
    //     }

    //     std::cout << "    joint " << i << " (" << name << ") is " << type << std::endl;
    //     i++;
    // }

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
            configuration,
            std::move(controller),
            simulator,
            robot,
            end_effector_frame_index,
            controller_countdown_max
        )
    );
}

FrankaRidgebackActor::FrankaRidgebackActor(
    const Configuration &configuration,
    std::unique_ptr<mppi::Trajectory> &&controller,
    Simulator *simulator,
    raisim::ArticulatedSystem *robot,
    std::size_t end_effector_index,
    std::int64_t controller_countdown_max
) : m_configuration(std::move(configuration))
  , m_simulator(simulator)
  , m_trajectory(std::move(controller))
  , m_robot(robot)
  , m_end_effector_frame_index(end_effector_index)
  , m_trajectory_countdown(0) // Update on first step.
  , m_trajectory_countdown_max(controller_countdown_max)
  , m_state(FrankaRidgeback::State::Zero())
  , m_external_joint_torques(FrankaRidgeback::DoF::JOINTS)
  , m_end_effector_jacobian(3, FrankaRidgeback::DoF::JOINTS)
  , m_energy_tank(configuration.energy)
{
    m_external_joint_torques.setZero();
    m_end_effector_jacobian.setZero();

    FrankaRidgeback::State state = m_trajectory->get_rolled_out_state();

    m_robot->setState(
        m_configuration.initial_state.position(),
        m_configuration.initial_state.velocity()
    );
}

void FrankaRidgebackActor::set_end_effector_force(
    Eigen::Ref<Eigen::Vector3d> force
) {
    m_robot->setExternalForce(
        m_robot->getFrameByIdx(m_end_effector_frame_index).parentId,
        raisim::ArticulatedSystem::Frame::WORLD_FRAME,
        force,
        raisim::ArticulatedSystem::Frame::BODY_FRAME,
        raisim::Vec<3>()
    );
}

void FrankaRidgebackActor::set_end_effector_torque(
    Eigen::Ref<Eigen::Vector3d> torque
) {
    m_robot->setExternalTorque(
        m_robot->getFrameByIdx(m_end_effector_frame_index).parentId,
        torque
    );
}

void FrankaRidgebackActor::act(Simulator *simulator)
{
    using namespace FrankaRidgeback;

    // Update the controller every couple of time steps, depending on the
    // controller update rate.
    if (--m_trajectory_countdown <= 0) {
        m_trajectory_countdown = m_trajectory_countdown_max;

        for (unsigned int i = 0; i < m_configuration.controller_substeps; i++)
            m_trajectory->update(m_state, simulator->get_time());
    }

    // Get the controls to apply to the robot joints for the current time.
    m_trajectory->get(m_control, simulator->get_time());

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

void FrankaRidgebackActor::update(Simulator *simulator)
{
    Eigen::VectorXd state_position, state_velocity;
    m_robot->getState(state_position, state_velocity);

    m_state.position() = state_position;
    m_state.velocity() = state_velocity;

    // Get the end effector jacobian.
    Eigen::MatrixXd linear {3, FrankaRidgeback::DoF::JOINTS};
    linear.setZero();
    m_robot->getDenseFrameJacobian(m_end_effector_frame_index, linear);
    m_end_effector_jacobian.topRows(3) = linear;

    Eigen::MatrixXd angular {3, FrankaRidgeback::DoF::JOINTS};
    angular.setZero();
    m_robot->getDenseFrameRotationalJacobian(m_end_effector_frame_index, angular);
    m_end_effector_jacobian.bottomRows(3) = angular;

    // Set the appropriate jacobian for the ridgeback.
    double yaw = m_state.base_yaw().value();
    m_end_effector_jacobian.topLeftCorner<3, 3>() <<
        std::cos(yaw), -std::sin(yaw), 0,
        std::sin(yaw), std::cos(yaw), 0,
        0, 0, 1;

    // Update the external joint torques.
    m_external_joint_torques = (
        m_end_effector_jacobian.transpose() * get_end_effector_force()
    );

    // Update the energy tank.
    m_energy_tank.step(
        // m_robot->getKineticEnergy() / m_simulator->get_time_step(),
        m_external_joint_torques.transpose() * state_velocity,
        m_simulator->get_time_step()
    );

    m_state.end_effector_force() = get_end_effector_force();
    m_state.end_effector_torque() = get_end_effector_torque();
    m_state.available_energy() = m_energy_tank.get_energy();
}
