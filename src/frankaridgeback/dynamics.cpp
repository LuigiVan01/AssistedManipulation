#include "dynamics.hpp"

#include <iostream>
#include <string>
#include <utility>

using namespace std::string_literals;

namespace FrankaRidgeback {

std::unique_ptr<Dynamics> Dynamics::create(
    const Configuration &configuration,
    ForcePredictor *force_predictor
) {
    auto model = FrankaRidgeback::Model::create(configuration.model);
    if (!model) {
        std::cout << "failed to create dynamics model." << std::endl;
        return nullptr;
    }

    return std::unique_ptr<Dynamics>(
        new Dynamics(
            configuration.energy,
            std::move(model),
            std::move(force_predictor->create_handle())
        )
    );
}

Dynamics::Dynamics(
    double energy,
    std::unique_ptr<FrankaRidgeback::Model> &&model,
    std::unique_ptr<ForcePredictor::Handle> &&force_predictor_handle
  ) : m_model(std::move(model))
    , m_energy_tank(energy)
    , m_state(State::Zero())
    , m_force_predictor_handle(std::move(force_predictor_handle))
{}

std::unique_ptr<mppi::Dynamics> Dynamics::copy()
{
    std::unique_ptr<ForcePredictor::Handle> predictor = nullptr;
    if (m_force_predictor_handle)
        predictor = m_force_predictor_handle->copy();

    return std::unique_ptr<Dynamics>(
        new Dynamics(
            m_energy_tank.get_energy(),
            std::move(m_model->copy()),
            std::move(predictor)
        )
    );
}

void Dynamics::set(const Eigen::VectorXd &state)
{
    m_model->set(state);
    m_energy_tank.set_energy(state.available_energy());
    m_state = state;
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

    m_force_predictor_handle->predict();

    // m_energy_tank.step(m_state.velocity() , dt);

    return m_state;
}

} // namespace FrankaRidgeback
