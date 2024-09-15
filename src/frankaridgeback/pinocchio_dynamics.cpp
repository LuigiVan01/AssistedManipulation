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
    , m_wrench_forecast(std::move(force_predictor_handle))
    , m_end_effector_index(end_effector_index)
    , m_end_effector_jacobian(6, FrankaRidgeback::DoF::JOINTS)
    , m_end_effector_velocity(6, 1)
    , m_energy_tank(energy)
    , m_time(0.0)
{
    m_position.setZero();
    m_velocity.setZero();
    m_torque.setZero();
    m_acceleration.setZero();
}

std::unique_ptr<mppi::Dynamics> PinocchioDynamics::copy()
{
    auto model = std::make_unique<pinocchio::Model>(*m_model);
    auto data = std::make_unique<pinocchio::Data>(*model);

    std::unique_ptr<Forecast::Handle> predictor = nullptr;
    if (m_wrench_forecast)
        predictor = m_wrench_forecast->copy();

    return std::unique_ptr<PinocchioDynamics>(
        new PinocchioDynamics(
            std::move(model),
            std::move(data),
            std::move(predictor),
            m_end_effector_index,
            m_energy_tank.get_energy()
        )
    );
}

void PinocchioDynamics::update_kinematics()
{
    // Update the robot joints with the previously cal.
    pinocchio::forwardKinematics(*m_model, *m_data, m_position, m_velocity);
    pinocchio::updateFramePlacements(*m_model, *m_data);

    // Calculate the external torque from the externally applied force.
    pinocchio::computeJointJacobians(*m_model, *m_data);
    pinocchio::getFrameJacobian(
        *m_model,
        *m_data,
        m_end_effector_index,
        pinocchio::ReferenceFrame::WORLD,
        m_end_effector_jacobian
    );

    // Update the base jacobian to be relative to the arm.
    double yaw = m_position[2];
    m_end_effector_jacobian.topLeftCorner<3, 3>()
        << std::cos(yaw), -std::sin(yaw), 0,
           std::sin(yaw), std::cos(yaw), 0,
           0, 0, 1;

    // Update the end effector velocity.
    m_end_effector_velocity = pinocchio::getFrameVelocity(
        *m_model,
        *m_data,
        m_end_effector_index,
        pinocchio::WORLD
    ).toVector();
}

void PinocchioDynamics::set(const Eigen::VectorXd &state, double time)
{
    m_time = time;
    m_state = state;
    m_position = m_state.position();
    m_velocity = m_state.velocity();
    m_energy_tank.set_energy(m_state.available_energy().value());

    update_kinematics();
}

Eigen::Ref<Eigen::VectorXd> PinocchioDynamics::step(const Eigen::VectorXd &ctrl, double dt)
{
    const Control &control = ctrl;

    m_time += dt;

    // The current yaw of the robot.
    double yaw = m_position[2];

    // Set the velocity control. Rotate base velocity to the robot frame of reference.
    m_velocity.head<2>() = Eigen::Rotation2Dd(yaw) * control.base_velocity();
    m_velocity[2] = control.base_angular_velocity().value();

    // Set the joint torque as the control torque + end effector wrench torque.
    m_torque.setZero();
    m_torque.segment<DoF::ARM>(DoF::BASE) = control.arm_torque();
    m_torque += m_end_effector_jacobian.transpose() * m_state.end_effector_wrench();

    /// @warning The following algorithm diverges quickly, and is not very
    /// debuggable. See simulation/dynamics.hpp for a raisim based dynamics.

    // Calculate the joint accelerations.
    m_acceleration = pinocchio::aba(
        *m_model.get(),
        *m_data.get(),
        m_position,
        m_velocity,
        m_torque
    );

    // Euler integrate the forward dynamics.
    m_velocity += m_acceleration * dt;
    m_position += m_velocity * dt;

    // Update the positions.
    update_kinematics();

    double power = m_torque.transpose() * m_velocity;
    m_energy_tank.step(power, dt);

    m_state.position() = m_position;
    m_state.velocity() = m_velocity;
    m_state.available_energy().setConstant(m_energy_tank.get_energy());

    if (m_wrench_forecast)
        m_state.end_effector_wrench() = m_wrench_forecast->forecast(m_time);
    else
        m_state.end_effector_wrench().setZero();

    return m_state;
}

} // namespace FrankaRidgeback
