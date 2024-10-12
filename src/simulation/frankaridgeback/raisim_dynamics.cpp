#include "simulation/frankaridgeback/raisim_dynamics.hpp"

namespace FrankaRidgeback {

std::unique_ptr<RaisimDynamics> RaisimDynamics::create(
    Configuration configuration,
    std::unique_ptr<DynamicsForecast::Handle> &&dynamics_forecast_handle,
    Simulator *simulator
) {
    if (configuration.filename.empty())
        configuration.filename = find_path().string();

    std::shared_ptr<raisim::World> world = nullptr;

    // For simulating the system as a whole (not using for mppi trajectory
    // generation), the dynamics references a world created outside of this
    // instance. In this case, integrating the world is handled externally.
    // When using the mppi trajectory generation, the dynamics requires its own
    // world to simulate rollouts in, based on the provided simulator
    // configuration.
    if (configuration.simulator) {
        world = std::make_shared<raisim::World>();
        world->setTimeStep(configuration.simulator->time_step);
        world->setGravity(configuration.simulator->gravity);
        world->addGround();
    }
    else {
        if (!simulator) {
            std::cerr << "provided simulator in raisim dynamics is nullptr" << std::endl;
            return nullptr;
        }
        world = simulator->get_world();
    }

    // Ensure computing the inverse dynamics is enabled to get the end effector
    // frame acceleration.
    auto frankaridgeback = world->addArticulatedSystem(configuration.filename);
    // frankaridgeback->setComputeInverseDynamics(true);
    frankaridgeback->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);

    std::size_t end_effector_frame_index = frankaridgeback->getFrameIdxByName(
        configuration.end_effector_frame
    );

    if (end_effector_frame_index > frankaridgeback->getFrames().size() - 1) {
        std::cerr << "end effector frame \""
                  << configuration.end_effector_frame
                  << "\" does not exist " << std::endl;
        std::cerr << "existing frames: ";
        for (auto &x : frankaridgeback->getFrames())
            std::cerr << x.name << ", ";
        std::cerr << std::endl;
        return nullptr;
    }

    frankaridgeback->setPdGains(
        configuration.proportional_gain,
        configuration.differential_gain
    );

    // Start with zero force on the joints.
    auto zero = VectorXd::Zero((Eigen::Index)frankaridgeback->getDOF());
    frankaridgeback->setGeneralizedForce(zero);

    return std::unique_ptr<RaisimDynamics>(
        new RaisimDynamics(
            configuration,
            std::move(world),
            std::move(dynamics_forecast_handle),
            frankaridgeback,
            end_effector_frame_index,
            configuration.end_effector_frame
        )
    );
}

RaisimDynamics::RaisimDynamics(
    const Configuration &configuration,
    std::shared_ptr<raisim::World> &&world,
    std::unique_ptr<DynamicsForecast::Handle> &&dynamics_forecast_handle,
    raisim::ArticulatedSystem *robot,
    std::int64_t end_effector_frame_index,
    std::string end_effector_frame_name
 ) : m_configuration(configuration)
   , m_world(std::move(world))
   , m_robot(robot)
   , m_position_command(DoF::CONTROL)
   , m_velocity_command(DoF::CONTROL)
   , m_position(DoF::CONTROL)
   , m_velocity(DoF::CONTROL)
   , m_end_effector_frame_index(end_effector_frame_index)
   , m_end_effector_frame_name(end_effector_frame_name)
   , m_end_effector_state()
   , m_end_effector_linear_jacobian(3, FrankaRidgeback::DoF::JOINTS)
   , m_end_effector_angular_jacobian(3, FrankaRidgeback::DoF::JOINTS)
   , m_joint_power(0.0)
   , m_external_power(0.0)
   , m_energy_tank(m_configuration.initial_state.available_energy().value())
   , m_state(configuration.initial_state)
   , m_forecast(std::move(dynamics_forecast_handle))
   , m_end_effector_simulated_wrench(Vector6d::Zero())
{
    State initial = configuration.initial_state;
    if (configuration.energy) {
        initial.available_energy().setConstant(configuration.energy.value());
        m_energy_tank.set_energy(configuration.energy.value());
    }

    m_position_command.setZero();
    m_velocity_command.setZero();
    m_position.setZero();
    m_velocity.setZero();
    m_end_effector_linear_jacobian.setZero();
    m_end_effector_angular_jacobian.setZero();

    set_state(initial, m_world->getWorldTime());
}

void RaisimDynamics::set_state(const VectorXd &state, double time)
{
    m_world->setWorldTime(time);

    m_state = state;
    m_energy_tank.set_energy(m_state.available_energy().value());
    m_robot->setState(m_state.position(), m_state.velocity());
    m_external_power = 0.0;

    calculate();
}

void RaisimDynamics::add_end_effector_simulated_wrench(Vector6d wrench)
{
    m_end_effector_simulated_wrench += wrench;
    auto body = m_robot->getFrameByIdx(m_end_effector_frame_index).parentId;

    // Set the external force on the end effector. Note that the end effector is
    // offset from the origin of the most proximal link.
    m_robot->setExternalForce(
        body,
        raisim::ArticulatedSystem::Frame::WORLD_FRAME,
        m_end_effector_simulated_wrench.head<3>(),
        raisim::ArticulatedSystem::Frame::BODY_FRAME,
        Vector3d(0.0, 0.0, 0.2)
    );

    // Torque is applied to the parent joint itself.
    m_robot->setExternalTorque(body, m_end_effector_simulated_wrench.tail<3>());
}

void RaisimDynamics::calculate()
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

    m_end_effector_state.jacobian.topRows(3) = m_end_effector_linear_jacobian;
    m_end_effector_state.jacobian.bottomRows(3) = m_end_effector_angular_jacobian;

    // Update the base jacobian to be relative to the arm.
    double yaw = m_state.base_yaw().value();
    m_end_effector_state.jacobian.topLeftCorner<3, 3>()
        << std::cos(yaw), -std::sin(yaw), 0,
           std::sin(yaw), std::cos(yaw), 0,
           0, 0, 1;

    /// TODO: Incorrect, power port is from the external wrench only.
    m_joint_power = (
        m_robot->getGeneralizedForce().e().transpose() *
        m_robot->getGeneralizedVelocity().e()
    ).value();

    // Update the power usage from the end effector torque.
    // if (!m_end_effector_wrench.isZero()) {
    //     m_external_torque = m_end_effector_jacobian.transpose() * get_end_effector_wrench();
    //     m_power = (m_state.velocity().transpose() * m_external_torque).value();
    // }
    // else {
    //     m_external_torque.setZero();
    // }

    // Update end effector position and orientation.
    raisim::Vec<3> position;
    m_robot->getFramePosition(m_end_effector_frame_index, position);
    m_end_effector_state.position = position.e();

    /// Update end effector orientation.
    raisim::Mat<3, 3> orientation;
    m_robot->getFrameOrientation(m_end_effector_frame_index, orientation);
    m_end_effector_state.orientation = Quaterniond(orientation.e());

    // Update the end effector velocity.
    raisim::Vec<3> end_effector_linear_velocity, end_effector_angular_velocity;
    auto frame = m_robot->getFrameByIdx(m_end_effector_frame_index);
    m_robot->getFrameVelocity(frame, end_effector_linear_velocity);
    m_robot->getAngularVelocity(frame.parentId, end_effector_angular_velocity);
    m_end_effector_state.linear_velocity = end_effector_linear_velocity.e();
    m_end_effector_state.angular_velocity = end_effector_angular_velocity.e();

    // Update the end effector acceleration.
    raisim::Vec<3> end_effector_linear_acceleration;
    m_robot->getFrameAcceleration(m_end_effector_frame_name, end_effector_linear_acceleration);
    m_end_effector_state.linear_acceleration = end_effector_linear_acceleration.e();
    m_end_effector_state.angular_acceleration.setZero();
}

void RaisimDynamics::act(const Control &control)
{
    // Gripper positional controls.
    m_position_command.setZero();
    m_position_command.tail<DoF::GRIPPER>() = m_state.gripper_position();
    // m_position_command.tail<DoF::GRIPPER>() = m_state.control.gripper_position();

    // Rotate the desired base velocity command into the base frame of reference.
    m_velocity_command.setZero();
    m_velocity_command.head<DoF::BASE_POSITION>() = /* Eigen::Rotation2Dd(m_state.base_yaw().value()) * */ control.base_velocity();
    m_velocity_command.segment<DoF::BASE_YAW>(DoF::BASE_POSITION) = control.base_angular_velocity();
    // m_velocity_command.segment<DoF::ARM>(DoF::BASE) = control.arm_velocity();

    // Set the PD controller targets and apply the torques.
    auto forces = m_robot->getNonlinearities(m_world->getGravity()).e().eval();
    forces.segment<DoF::ARM>(DoF::BASE) += control.arm_velocity();

    m_robot->setPdTarget(m_position_command, m_velocity_command);
    m_robot->setGeneralizedForce(forces);

    m_external_power = (
        control.transpose() *
        (m_end_effector_state.jacobian.transpose() * m_end_effector_simulated_wrench)
    );

    // We could simulate the forecasted end effector wrench, but this would make
    // the calculation of the user observed end effector force incorrect.
}

void RaisimDynamics::update()
{
    m_end_effector_simulated_wrench.setZero();

    calculate();

    // Update position and velocity.
    m_state.position() = m_position;
    m_state.velocity() = m_velocity;

    // Update the energy tank.
    m_energy_tank.step(-m_external_power, m_world->getTimeStep());
    m_state.available_energy().setConstant(m_energy_tank.get_energy());
}

Eigen::Ref<VectorXd> RaisimDynamics::step(
    const VectorXd &control,
    double dt
) {
    act(control);
    m_world->integrate();
    update();

    return m_state;
}

} // namespace FrankaRidgeback
