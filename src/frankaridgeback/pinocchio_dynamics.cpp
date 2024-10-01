#include "frankaridgeback/pinocchio_dynamics.hpp"

#include <filesystem>
#include <iostream>
#include <string>
#include <utility>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

using namespace std::string_literals;

namespace FrankaRidgeback {

const PinocchioDynamics::Configuration PinocchioDynamics::DEFAULT_CONFIGURATION {
    .filename = "",
    .end_effector_frame = "panda_grasp_joint",
    .initial_state = FrankaRidgeback::State::Zero(),
    .energy = 10.0
};

std::unique_ptr<PinocchioDynamics> PinocchioDynamics::create(
    Configuration configuration,
    std::unique_ptr<Forecast::Handle> &&wrench_forecast_handle
) {
    if (configuration.filename.empty())
        configuration.filename = find_path().string();

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

    // Build geometry: https://docs.ros.org/en/noetic/api/pinocchio/html/namespacepinocchio_1_1urdf.html

    std::unique_ptr<pinocchio::Model> model;
    std::unique_ptr<pinocchio::Data> data;
    std::unique_ptr<pinocchio::GeometryModel> geometry_model;
    std::unique_ptr<pinocchio::GeometryData> geometry_data;
    try {
        model = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModelFromXML(urdf.str(), *model);
        data = std::make_unique<pinocchio::Data>(*model);

        geometry_model = std::make_unique<pinocchio::GeometryModel>();
        pinocchio::urdf::buildGeom(
            *model,
            configuration.filename,
            pinocchio::GeometryType::COLLISION,
            *geometry_model
        );
        geometry_data = std::make_unique<pinocchio::GeometryData>(*geometry_model);
    }
    catch (const std::exception &err) {
        std::cout << "failed to create model. " << err.what() << std::endl;
        return nullptr;
    }

    auto end_effector_frame_index = model->getFrameId(configuration.end_effector_frame);

    return std::unique_ptr<PinocchioDynamics>(
        new PinocchioDynamics(
            configuration,
            std::move(model),
            std::move(data),
            std::move(geometry_model),
            std::move(geometry_data),
            std::move(wrench_forecast_handle),
            end_effector_frame_index
        )
    );
}

PinocchioDynamics::PinocchioDynamics(
    const Configuration &configuration,
    std::unique_ptr<pinocchio::Model> &&model,
    std::unique_ptr<pinocchio::Data> &&data,
    std::unique_ptr<pinocchio::GeometryModel> &&geometry_model,
    std::unique_ptr<pinocchio::GeometryData> &&geometry_data,
    std::unique_ptr<Forecast::Handle> &&force_predictor_handle,
    std::size_t end_effector_frame_index
  ) : m_configuration(configuration)
    , m_model(std::move(model))
    , m_data(std::move(data))
    , m_geometry_model(std::move(geometry_model))
    , m_geometry_data(std::move(geometry_data))
    , m_end_effector_frame_index(end_effector_frame_index)
    , m_joint_position()
    , m_joint_velocity()
    , m_joint_torque()
    , m_joint_acceleration()
    , m_end_effector_jacobian(Jacobian::Zero())
    , m_end_effector_spatial_velocity(Vector6d::Zero())
    , m_end_effector_spatial_acceleration(Vector6d::Zero())
    , m_forecast(std::move(force_predictor_handle))
    , m_end_effector_forecast_wrench(Vector6d::Zero())
    , m_end_effector_virtual_wrench(Vector6d::Zero())
    , m_power(0.0)
    , m_energy_tank(configuration.energy)
    , m_state()
    , m_time(0.0)
{
    m_joint_position.setZero();
    m_joint_velocity.setZero();
    m_joint_torque.setZero();
    m_joint_acceleration.setZero();

    set_state(configuration.initial_state, m_time);
}

std::unique_ptr<mppi::Dynamics> PinocchioDynamics::copy()
{
    auto model = std::make_unique<pinocchio::Model>(*m_model);
    auto data = std::make_unique<pinocchio::Data>(*model);
    auto geometry_model = std::make_unique<pinocchio::GeometryModel>(*m_geometry_model);
    auto geometry_data = std::make_unique<pinocchio::GeometryData>(*m_geometry_data);

    std::unique_ptr<Forecast::Handle> predictor = nullptr;
    if (m_forecast)
        predictor = m_forecast->copy();

    return std::unique_ptr<PinocchioDynamics>(
        new PinocchioDynamics(
            m_configuration,
            std::move(model),
            std::move(data),
            std::move(geometry_model),
            std::move(geometry_data),
            std::move(predictor),
            m_end_effector_frame_index
        )
    );
}

void PinocchioDynamics::set_state(const Eigen::VectorXd &state, double time)
{
    m_time = time;
    m_state = state;
    m_joint_position = m_state.position();
    m_joint_velocity = m_state.velocity();
    m_energy_tank.set_energy(m_state.available_energy().value());

    calculate();
}

void PinocchioDynamics::calculate()
{
    // Gravity compensation.
    m_joint_torque += pinocchio::nonLinearEffects(
        *m_model.get(),
        *m_data.get(),
        m_joint_position,
        m_joint_velocity
    );

    // Calculate the joint accelerations.
    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/topic/doc-v2/doxygen-html/namespacese3.html#a60ee959532506ecafc6790cadce323a2
    m_joint_acceleration = pinocchio::aba(
        *m_model.get(),
        *m_data.get(),
        m_joint_position,
        m_joint_velocity,
        m_joint_torque
    );

    // Second order forward kinematics for acceleration calculations.
    pinocchio::forwardKinematics(
        *m_model,
        *m_data,
        m_joint_position,
        m_joint_velocity,
        m_joint_acceleration
    );

    pinocchio::updateFramePlacements(*m_model, *m_data);

    // Get the end effector jacobian.
    m_end_effector_jacobian.setZero();
    pinocchio::computeFrameJacobian(
        *m_model.get(),
        *m_data.get(),
        m_joint_position,
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

    m_end_effector_kinematics.position = m_data->oMf[m_end_effector_frame_index].translation();
    m_end_effector_kinematics.orientation = m_data->oMf[m_end_effector_frame_index].rotation();
    m_end_effector_kinematics.linear_velocity = m_end_effector_spatial_velocity.head<3>();
    m_end_effector_kinematics.angular_velocity = m_end_effector_spatial_velocity.tail<3>();
}

Eigen::Ref<Eigen::VectorXd> PinocchioDynamics::step(const Eigen::VectorXd &c, double dt)
{
    const Control &control = c;

    // The current yaw of the robot.
    double yaw = m_joint_position[2];

    // Set the velocity control. Rotate base velocity to the robot frame of reference.
    m_joint_velocity.head<2>() = Eigen::Rotation2Dd(yaw) * control.base_velocity();
    m_joint_velocity[2] = control.base_angular_velocity().value();

    // Set the joint torque as the control torque + end effector wrench torque.
    m_joint_torque.setZero();
    m_joint_torque.segment<DoF::ARM>(DoF::BASE) = control.arm_velocity();
    // m_joint_torque += m_end_effector_jacobian.transpose() * m_state.end_effector_wrench();

    calculate();

    // Euler integrate the forward dynamics.
    m_joint_velocity += m_joint_acceleration * dt;
    m_joint_position += m_joint_velocity * dt;

    // Integrate the energy tank with current power consumption.
    m_power = m_joint_torque.transpose() * m_joint_velocity;
    m_energy_tank.step(m_power, dt);
    m_state.available_energy().setConstant(m_energy_tank.get_energy());

    // Update the time of the wrench forecast.
    m_time += dt;

    if (m_forecast) {
        m_end_effector_virtual_wrench = calculate_virtual_end_effector_wrench();
        m_end_effector_forecast_wrench = m_forecast->forecast(m_time);
        m_state.end_effector_wrench() = m_end_effector_forecast_wrench - m_end_effector_virtual_wrench;
    }
    else {
        m_state.end_effector_wrench().setZero();
    }

    m_state.position() = m_joint_position;
    m_state.velocity() = m_joint_velocity;

    return m_state;
}

Vector6d PinocchioDynamics::calculate_virtual_end_effector_wrench()
{
    return Vector6d::Zero();
}

} // namespace FrankaRidgeback
