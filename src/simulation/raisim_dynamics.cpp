#include "simulation/raisim_dynamics.hpp"

namespace FrankaRidgeback {

std::unique_ptr<RaisimDynamics> RaisimDynamics::create(
    const Configuration &configuration,
    std::unique_ptr<Forecast::Handle> &&forecast
) {
    auto world = std::make_unique<raisim::World>();
    world->setTimeStep(configuration.simulator.time_step);
    world->setGravity(configuration.simulator.gravity);
    world->addGround();

    auto robot = world->addArticulatedSystem(configuration.filename);
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

    robot->setPdGains(
        configuration.proportional_gain,
        configuration.differential_gain
    );

    // Start with zero force on the joints.
    robot->setGeneralizedForce(
        Eigen::VectorXd::Zero((Eigen::Index)robot->getDOF())
    );

    return std::unique_ptr<RaisimDynamics>(
        new RaisimDynamics(
            configuration,
            std::move(world),
            std::move(forecast),
            robot,
            end_effector_frame_index
        )
    );
}

RaisimDynamics::RaisimDynamics(
    const Configuration &configuration,
    std::unique_ptr<raisim::World> &&world,
    std::unique_ptr<Forecast::Handle> &&forecast_handle,
    raisim::ArticulatedSystem *robot,
    std::int64_t end_effector_frame_index
 ) : m_configuration(configuration)
   , m_world(std::move(world))
   , m_robot(robot)
   , m_position_command(DoF::CONTROL)
   , m_velocity_command(DoF::CONTROL)
   , m_position(DoF::CONTROL)
   , m_velocity(DoF::CONTROL)
   , m_external_torque(DoF::CONTROL)
   , m_forecast(std::move(forecast_handle))
   , m_end_effector_frame_index(end_effector_frame_index)
   , m_end_effector_linear_jacobian(3, FrankaRidgeback::DoF::JOINTS)
   , m_end_effector_angular_jacobian(3, FrankaRidgeback::DoF::JOINTS)
   , m_end_effector_jacobian(6, FrankaRidgeback::DoF::JOINTS)
   , m_end_effector_velocity(Eigen::Vector3d::Zero())
   , m_external_end_effector_force(Eigen::Vector3d::Zero())
   , m_power(0.0)
   , m_energy_tank(configuration.energy)
   , m_state(configuration.initial_state)
   , m_time(0.0)
{
    m_position_command.setZero();
    m_velocity_command.setZero();
    m_position.setZero();
    m_velocity.setZero();
    set_state(configuration.initial_state, m_world->getWorldTime());
}

void RaisimDynamics::set_state(
    const Eigen::VectorXd &state,
    double time
) {
    m_state = state;
    m_energy_tank.set_energy(m_state.available_energy().value());

    m_robot->setState(m_state.position(), m_state.velocity());
    update_kinematics();
}

Eigen::Ref<Eigen::VectorXd> RaisimDynamics::step(
    const Eigen::VectorXd &c,
    double dt
) {
    const Control &control = c;

    // Gripper positional controls.
    m_position_command.tail<DoF::GRIPPER>() = control.gripper_position();

    // Rotate the desired base velocity command into the base frame of reference.
    m_velocity_command.head<DoF::BASE_POSITION>() = (
        Eigen::Rotation2Dd(m_state.base_yaw().value()) * control.base_velocity()
    );

    // Set the desired base rotational and arm velocities.
    m_velocity_command.segment<DoF::BASE_YAW>(DoF::BASE_POSITION) = control.base_angular_velocity();
    m_velocity_command.segment<DoF::ARM>(DoF::BASE) = control.arm_velocity();

    // Set the PD controller targets and apply the torques.
    m_robot->setPdTarget(m_position_command, m_velocity_command);
    m_robot->setGeneralizedForce(m_robot->getNonlinearities(m_world->getGravity()));

    // Simulator the predicted external force.
    if (m_forecast) {
        m_external_end_effector_force += m_forecast->forecast(m_world->getWorldTime());
    }

    m_world->integrate();

    m_robot->setExternalForce(
        m_robot->getFrameByIdx(m_end_effector_frame_index).parentId,
        m_external_end_effector_force
    );

    update_kinematics();

    // Update the energy tank.
    m_energy_tank.step(m_power, m_world->getTimeStep());

    m_robot->getState(m_position, m_velocity);
    m_state.position() = m_position;
    m_state.velocity() = m_velocity;
    m_state.end_effector_force() = m_external_end_effector_force;
    m_state.end_effector_torque().setZero();
    m_state.available_energy().setConstant(m_energy_tank.get_energy());

    // Clear the external force.
    m_external_end_effector_force.setZero();

    return m_state;
}

void RaisimDynamics::update_kinematics()
{
    m_robot->getState(m_position, m_velocity);

    m_end_effector_linear_jacobian.setZero();
    m_robot->getDenseFrameRotationalJacobian(
        m_end_effector_frame_index,
        m_end_effector_linear_jacobian
    );

    m_end_effector_angular_jacobian.setZero();
    m_robot->getDenseFrameRotationalJacobian(
        m_end_effector_frame_index,
        m_end_effector_angular_jacobian
    );

    m_end_effector_jacobian.topRows(3) = m_end_effector_linear_jacobian;
    m_end_effector_jacobian.bottomRows(3) = m_end_effector_angular_jacobian;

    // Update the base jacobian to be relative to the arm.
    double yaw = m_state.base_yaw().value();
    m_end_effector_jacobian.topLeftCorner<3, 3>()
        << std::cos(yaw), -std::sin(yaw), 0,
           std::sin(yaw), std::cos(yaw), 0,
           0, 0, 1;

    // Get the torques on joints from external force.
    if (!m_external_end_effector_force.isZero())
        m_external_torque = m_end_effector_linear_jacobian.transpose() * get_end_effector_force();
    else
        m_external_torque.setZero();

    // Update the power usage.
    m_power = (m_state.velocity().transpose() * m_external_torque).value();

    // Update the end effector velocity.
    raisim::Vec<3> velocity;
    m_robot->getFrameVelocity(
        m_robot->getFrameByIdx(m_end_effector_frame_index),
        velocity
    );
    m_end_effector_velocity = velocity.e();
}

} // namespace FrankaRidgeback
