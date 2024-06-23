#include "dynamics.hpp"

#include <iostream>
#include <string>
#include <utility>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

using namespace std::string_literals;

namespace FrankaRidgeback {

Dynamics::Dynamics()
    : m_state(State::Zero())
{}

std::shared_ptr<Dynamics> Dynamics::create()
{
    return std::shared_ptr<Dynamics>(new Dynamics());
}

Eigen::Ref<Eigen::VectorXd> Dynamics::step(const Eigen::VectorXd &ctrl, double dt)
{
    const Control &control = ctrl;

    double yaw = m_state.base_yaw().value();
    auto rotated = (Eigen::Rotation2Dd(yaw) * control.base_velocity()).eval();

    // Rotate base velocity to the robot frame of reference.
    m_state.base_position() = rotated * dt;
    m_state.base_yaw() += control.base_angular_velocity() * dt;

    // Double integrate arm torque to position. Should use a better numerical
    // method.
    m_state.arm_velocity() += control.arm_torque() * dt;
    m_state.arm_position() += m_state.arm_velocity() * dt;

    // Gripper, usually redundant.
    // m_state.gripper_position() = control.gripper_position();

    return m_state;
}

std::unique_ptr<Model> Model::create(
    const std::string &filename,
    const std::string &end_effector_frame
) {
    // Open the file.
    std::ifstream file {filename, std::ios::in};

    // Check file exists and is readable.
    if (!file.is_open()) {
        std::cerr << "failed to open file \"" << filename << "\" for model" << std::endl;
        return nullptr;
    }

    // Read the file.
    std::stringstream urdf;
    urdf << file.rdbuf();
    file.close();

    if (file.bad() || file.fail()) {
        std::cerr << "failed to read file \"" << filename << "\" for model" << std::endl;
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

    auto end_effector_index = model->getFrameId(end_effector_frame);

    return std::unique_ptr<Model>(
        new Model(
            std::move(model),
            std::move(data),
            end_effector_index
        )
    );
}

Model::Model(
    std::unique_ptr<pinocchio::Model> model,
    std::unique_ptr<pinocchio::Data> data,
    std::size_t end_effector_index
  ) : m_model(std::move(model))
    , m_data(std::move(data))
    , m_end_effector_index(end_effector_index)
{}

void Model::update(const State &state)
{
    pinocchio::forwardKinematics(*m_model, *m_data, state.position());
    pinocchio::updateFramePlacements(*m_model, *m_data);
}

// void Model::update(
//     const State &state,
//     const Velocity &velocity
// ) {
//     pinocchio::forwardKinematics(m_model, m_data, state, velocity);
//     pinocchio::updateFramePlacements(m_model, m_data);
// }

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> Model::end_effector()
{
    // oMf is vector of absolute frame placements in the space frame.
    return std::make_tuple(
        m_data->oMf[m_end_effector_index].translation(),
        (Eigen::Quaterniond)m_data->oMf[m_end_effector_index].rotation()
    );
}

} // namespace FrankaRidgeback
