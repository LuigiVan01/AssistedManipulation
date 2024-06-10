#include "dynamics.hpp"

#include <iostream>
#include <string>

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

const Dynamics::State &Dynamics::step(const Control &control, double dt)
{
    // integrate joint velocities
    m_state.tail<7>() += control.tail<7>() * dt;

    // Base velocity in in body frame
    const double &vx = control(0);
    // const double &vy = control(1);
    const double &yawd = control(2);
    const double &yaw = m_state(2);

    // if (holonomic_) {
        // m_state(0) += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
        // m_state(1) += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    // } else {
        m_state(0) += vx * std::cos(yaw) * dt;
        m_state(1) += vx * std::sin(yaw) * dt;
    // }

    m_state(2) += yawd * dt;
    return m_state;
}

std::unique_ptr<Model> Model::create(const std::string &filename)
{
    // Open the file.
    std::ifstream file {filename, std::ios::in};

    // Check file exists and is readable.
    if (!file.is_open()) {
        std::cerr << "failed to open file \"" << filename << "\" for model" << std::endl;
        return nullptr;
    }

    // Read the file.
    std::stringstream ss;
    ss << file.rdbuf();
    file.close();

    if (file.bad() || file.fail()) {
        std::cerr << "failed to read file \"" << filename << "\" for model" << std::endl;
        return nullptr;
    }

    std::string urdf = ss.str();

    std::unique_ptr<pinocchio::Model> model;
    std::unique_ptr<pinocchio::Data> data;
    try {
        model = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModelFromXML(urdf, *model);
        data = std::make_unique<pinocchio::Data>(*model);
    }
    catch (const std::exception &err) {
        std::cout << "failed to create model. " << err.what() << std::endl;
        return nullptr;
    }

    return std::unique_ptr<Model>(
        new Model(
            std::move(model),
            std::move(data),
            model->getFrameId("panda_grasp")
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
    pinocchio::forwardKinematics(*m_model, *m_data, state);
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
    return std::make_tuple(
        m_data->oMf[m_end_effector_index].translation(),
        (Eigen::Quaterniond)m_data->oMf[m_end_effector_index].rotation()
    );
}

} // namespace FrankaRidgeback
