#include "PinocchioDynamics.hpp"

#include <filesystem>
#include <iostream>
#include <string>
#include <utility>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/parsers/urdf.hpp>

using namespace std::string_literals;

namespace FrankaRidgeback {

std::unique_ptr<PinocchioDynamics> PinocchioDynamics::create(
    const Configuration &configuration,
    Forecast *force_predictor
) {
    // Open the file.
    std::ifstream file {configuration.filename, std::ios::in};

    // Check file exists and is readable.
    if (!file.is_open()) {
        std::cerr << "failed to open file \"" << configuration.filename << "\" for model" << std::endl;
        return nullptr;
    }

    // Read the file.
    std::stringstream urdf;
    urdf << file.rdbuf();
    file.close();

    if (file.bad() || file.fail()) {
        std::cerr << "failed to read file \"" << configuration.filename << "\" for model" << std::endl;
        return nullptr;
    }

    std::unique_ptr<pinocchio::Model> model;
    std::unique_ptr<pinocchio::Data> data;
    try {
        model = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModelFromXML(urdf.str(), *model);
        data = std::make_unique<pinocchio::Data>(*model);
    }
    catch (const std::exception &err) {
        std::cout << "failed to create model. " << err.what() << std::endl;
        return nullptr;
    }

    auto end_effector_index = model->getFrameId(configuration.end_effector_frame);

    std::unique_ptr<Forecast::Handle> handle = nullptr;
    if (force_predictor)
        handle = force_predictor->create_handle();

    return std::unique_ptr<PinocchioDynamics>(
        new PinocchioDynamics(
            std::move(model),
            std::move(data),
            std::move(handle),
            end_effector_index,
            configuration.energy
        )
    );
}

PinocchioDynamics::PinocchioDynamics(
    std::unique_ptr<pinocchio::Model> &&model,
    std::unique_ptr<pinocchio::Data> &&data,
    std::unique_ptr<Forecast::Handle> &&force_predictor_handle,
    std::size_t end_effector_index,
    double energy
  ) : m_model(std::move(model))
    , m_data(std::move(data))
    , m_forecast(std::move(force_predictor_handle))
    , m_end_effector_frame_index(end_effector_index)
    , m_end_effector_jacobian(Jacobian::Zero())
    , m_end_effector_spatial_velocity(Vector6d::Zero())
    , m_energy_tank(energy)
    , m_time(0.0)
{
    m_joint_position.setZero();
    m_joint_velocity.setZero();
    m_joint_torque.setZero();
    m_joint_acceleration.setZero();
}

std::unique_ptr<mppi::Dynamics> PinocchioDynamics::copy()
{
    auto model = std::make_unique<pinocchio::Model>(*m_model);
    auto data = std::make_unique<pinocchio::Data>(*model);

    std::unique_ptr<Forecast::Handle> predictor = nullptr;
    if (m_forecast)
        predictor = m_forecast->copy();

    return std::unique_ptr<PinocchioDynamics>(
        new PinocchioDynamics(
            std::move(model),
            std::move(data),
            std::move(predictor),
            m_end_effector_frame_index,
            m_energy_tank.get_energy()
        )
    );
}

void PinocchioDynamics::set(const Eigen::VectorXd &state, double time)
{
    m_time = time;
    m_state = state;
    m_energy_tank.set_energy(m_state.available_energy().value());

    update_kinematics();
}

void PinocchioDynamics::update_kinematics()
{
    // Update the robot joints with the previously cal.
    pinocchio::forwardKinematics(
        *m_model,
        *m_data,
        m_state.position(),
        m_state.velocity()
    );

    pinocchio::updateFramePlacements(*m_model, *m_data);

    // Calculate the external torque from the externally applied force.
    pinocchio::computeJointJacobians(*m_model, *m_data);
    pinocchio::getFrameJacobian(
        *m_model,
        *m_data,
        m_end_effector_frame_index,
        pinocchio::ReferenceFrame::WORLD,
        m_end_effector_jacobian
    );

    // Update the base jacobian to be relative to the arm.
    double yaw = m_joint_position[2];
    m_end_effector_jacobian.topLeftCorner<3, 3>()
        << std::cos(yaw), -std::sin(yaw), 0,
           std::sin(yaw), std::cos(yaw), 0,
           0, 0, 1;

    // Update the end effector velocity.
    m_end_effector_spatial_velocity = pinocchio::getFrameVelocity(
        *m_model,
        *m_data,
        m_end_effector_frame_index,
        pinocchio::WORLD
    ).toVector();

    // Update the end effector acceleration.
    m_end_effector_spatial_acceleration = pinocchio::getFrameAcceleration(
        *m_model,
        *m_data,
        m_end_effector_frame_index,
        pinocchio::WORLD
    ).toVector();
}

Eigen::Ref<Eigen::VectorXd> PinocchioDynamics::step(
    const Eigen::VectorXd &c,
    double dt
) {
    const Control &control = ctrl;

    // Rotate the base velocity from to the ridgeback frame of reference to the
    // world frame.
    double yaw = m_state.base_yaw().value();
    auto rotated_base_velocity = (Eigen::Rotation2Dd(yaw) * control.base_velocity()).eval();

    // Rotate base velocity to the robot frame of reference.
    m_state.base_position() += rotated_base_velocity * dt;
    m_state.base_yaw() += control.base_angular_velocity() * dt;

    // Double integrate arm torque to position. Should use a better numerical
    // method.
    m_state.arm_position() += control.arm_velocity() * dt;

    update_kinematics();

    double power = m_joint_torque.transpose() * m_joint_velocity;
    m_energy_tank.step(power, dt);

    m_state.position() = m_joint_position;
    m_state.velocity() = m_joint_velocity;
    m_state.available_energy().setConstant(m_energy_tank.get_energy());

    if (m_forecast)
        m_state.end_effector_wrench() = m_forecast->forecast(m_time);
    else
        m_state.end_effector_wrench().setZero();

    return m_state;
}

Eigen::Ref<Eigen::VectorXd> PinocchioDynamics::step_aba(const Eigen::VectorXd &ctrl, double dt)
{
    // /// The current joint positions.
    // Eigen::Vector<double, DoF::JOINTS> m_joint_position;

    // /// The current joint velocities.
    // Eigen::Vector<double, DoF::JOINTS> m_joint_velocity;

    // /// The current torques applied by the controller to the joints.
    // Eigen::Vector<double, DoF::JOINTS> m_joint_torque;

    // /// The acceleration of the joints from forward dynamics.
    // Eigen::Vector<double, DoF::JOINTS> m_joint_acceleration;

    const Control &control = ctrl;

    m_time += dt;

    // The current yaw of the robot.
    double yaw = m_joint_position[2];

    // Set the velocity control. Rotate base velocity to the robot frame of reference.
    m_joint_velocity.head<2>() = Eigen::Rotation2Dd(yaw) * control.base_velocity();
    m_joint_velocity[2] = control.base_angular_velocity().value();

    // Set the joint torque as the control torque + end effector wrench torque.

    // The following algorithm diverges quickly, and is not very debuggable.
    // m_joint_torque.setZero();
    // m_joint_torque.segment<DoF::ARM>(DoF::BASE) = control.arm_joint_torque();
    // m_joint_torque = m_end_effector_jacobian.transpose() * m_state.end_effector_wrench();

    // Calculate the joint accelerations.
    // m_joint_acceleration = pinocchio::aba(
    //     *m_model.get(),
    //     *m_data.get(),
    //     m_joint_position,
    //     m_joint_velocity,
    //     m_joint_torque
    // );

    // Euler integrate the forward dynamics.
    // m_joint_velocity += m_joint_acceleration * dt;
    // m_joint_position += m_joint_velocity * dt;

    // Update the positions.
    update_kinematics();

    double power = m_joint_torque.transpose() * m_joint_velocity;
    m_energy_tank.step(power, dt);

    m_state.position() = m_joint_position;
    m_state.velocity() = m_joint_velocity;
    m_state.available_energy().setConstant(m_energy_tank.get_energy());

    if (m_forecast)
        m_state.end_effector_wrench() = m_forecast->forecast(m_time);
    else
        m_state.end_effector_wrench().setZero();

    return m_state;
}

} // namespace FrankaRidgeback
