#include "frankaridgeback/model.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace FrankaRidgeback {

std::unique_ptr<Model> Model::create(const Configuration &configuration)
{
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

    return std::unique_ptr<Model>(
        new Model(
            configuration,
            std::move(model),
            std::move(data),
            end_effector_index
        )
    );
}

Model::Model(
    const Configuration &configuration,
    std::unique_ptr<pinocchio::Model> model,
    std::unique_ptr<pinocchio::Data> data,
    std::size_t end_effector_index
  ) : m_configuration(configuration)
    , m_model(std::move(model))
    , m_data(std::move(data))
    , m_end_effector_index(end_effector_index)
    , m_end_effector_jacobian(6, FrankaRidgeback::DoF::JOINTS)
    , m_end_effector_velocity(6, 1)
{}

void Model::set(const State &state)
{
    // Update the kinematics model.
    pinocchio::forwardKinematics(*m_model, *m_data, state.position(), state.velocity());
    pinocchio::updateFramePlacements(*m_model, *m_data);

    // Get the jacobian of end effector frame.
    pinocchio::computeJointJacobians(*m_model, *m_data);
    pinocchio::getFrameJacobian(
        *m_model,
        *m_data,
        m_end_effector_index,
        pinocchio::ReferenceFrame::WORLD,
        m_end_effector_jacobian
    );

    std::cout << m_end_effector_jacobian << std::endl;

    double yaw = state.base_yaw().value();
    m_end_effector_jacobian.topLeftCorner<3, 3>()
        << std::cos(yaw), -std::sin(yaw), 0,
            std::sin(yaw), std::cos(yaw), 0,
            0, 0, 1;

    m_end_effector_velocity = pinocchio::getFrameVelocity(
        *m_model,
        *m_data,
        m_end_effector_index,
        pinocchio::WORLD
    ).toVector();

    // pinocchio::computeStaticTorque(
    //     m_model.get(),
    //     m_data.get(),
    //     state.position(),
    //     state.end_effector_force()
    // );
    // pinocchio::calc_aba(m_model.get(), m_data.get(), );
    // pinocchio::get

    // // The joint torques is tau = J^T F_tip
    // m_joint_torques = m_end_effector_jacobian.transpose() * state.end_effector_force();
}

} // namespace FrankaRidgeback 
