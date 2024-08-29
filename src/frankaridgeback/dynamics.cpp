#include "dynamics.hpp"

#include <iostream>
#include <string>
#include <utility>

using namespace std::string_literals;

namespace FrankaRidgeback {

Dynamics::Dynamics(
    double energy,
    std::unique_ptr<FrankaRidgeback::Model> &&model
  ) : m_model(std::move(model))
    , m_energy_tank(energy)
    , m_state(State::Zero())
{}

std::unique_ptr<Dynamics> Dynamics::create(const Configuration &configuration)
{
    auto model = FrankaRidgeback::Model::create(configuration.model);
    if (!model) {
        std::cout << "failed to create dynamics model." << std::endl;
        return nullptr;
    }

    return std::unique_ptr<Dynamics>(
        new Dynamics(
            configuration.energy,
            std::move(model)
        )
    );
}

Eigen::Ref<Eigen::VectorXd> Dynamics::step(const Eigen::VectorXd &ctrl, double dt)
{
    const Control &control = ctrl;

    double yaw = m_state.base_yaw().value();
    auto rotated = (Eigen::Rotation2Dd(yaw) * control.base_velocity()).eval();

    // Rotate base velocity to the robot frame of reference.
    m_state.base_position() += rotated * dt;
    m_state.base_yaw() += control.base_angular_velocity() * dt;

    // Double integrate arm torque to position. Should use a better numerical
    // method.
    m_state.arm_velocity() += control.arm_torque() * dt;
    m_state.arm_position() += m_state.arm_velocity() * dt;

    // Gripper, usually redundant.
    // m_state.gripper_position() = control.gripper_position();

    // m_state.end_effector_force().setZero();
    // m_state.end_effector_torque().setZero();

    m_model->set(m_state);

    // m_energy_tank.step(m_state.velocity() , dt);

    return m_state;
}

} // namespace FrankaRidgeback
