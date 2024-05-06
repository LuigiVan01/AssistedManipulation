#include "application/dynamics.hpp"

State FrankaResearch3RidgebackDyanamics::step(const Control &control, double dt)
{
    // integrate joint velocities
    m_state.tail<7>() += control.tail<7>() * dt;

    // Base velocity in in body frame
    const double &vx = control(0);
    const double &vy = control(1);
    const double &yawd = control(2);
    const double &yaw = m_state(2);

    if (holonomic_) {
        m_state(0) += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
        m_state(1) += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    } else {
        m_state(0) += vx * std::cos(yaw) * dt;
        m_state(1) += vx * std::sin(yaw) * dt;
    }

    m_state(2) += yawd * dt;
    return m_state;
}

FrankaResearch3Ridgeback::FrankaResearch3Ridgeback(const std::string &urdf)
{
    m_model = std::make_unique<pinocchio::Model>(urdf);
    pinocchio::urdf::buildModelFromXML(urdf, m_model.get());
    m_data = std::make_unique<pinocchio::Data>(m_model.get());
    m_end_effector_index = m_model->GetFrameID("panda_grasp");
}

void FrankaResearch3Ridgeback::update(const State &state)
{
    pinocchio::forwardKinematics(m_model.get(), m_data.get(), state);
    pinocchio::updateFramePlacements(m_model.get(), m_data);
}

void FrankaResearch3Ridgeback::update(
    const State &state,
    const Velocity &velocity
) {
    pinocchio::forwardKinematics(m_model, m_data, state, velocity);
    pinocchio::updateFramePlacements(m_model, m_data);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> FrankaResearch3Ridgeback::end_effector()
{
    return std::make_tuple(
        m_data->oMf[m_end_effector_index].translation(),
        m_data->oMf[m_end_effector_index].rotation()
    );
}
