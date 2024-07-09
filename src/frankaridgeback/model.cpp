#include "frankaridgeback/model.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace FrankaRidgeback {

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

void Model::set(const State &state) {
    pinocchio::forwardKinematics(*m_model, *m_data, state.position());
    pinocchio::updateFramePlacements(*m_model, *m_data);
}

Eigen::Vector3d Model::offset(
    const std::string &from_frame,
    const std::string &to_frame
) {
    return m_data->oMf[m_model->getFrameId(to_frame)].translation() -
    m_data->oMf[m_model->getFrameId(from_frame)].translation();
}

Eigen::Matrix<double, 6, 1> Model::error(
    const std::string &from_frame,
    const std::string &to_frame
) {
    using namespace pinocchio;

    return log6(
        m_data->oMf[m_model->getFrameId(to_frame)].actInv(
            m_data->oMf[m_model->getFrameId(from_frame)]
        )
    ).toVector();
}

Eigen::Matrix<double, 6, 1> Model::error(
    const std::string &frame,
    const Eigen::Quaterniond& rot,
    const Eigen::Vector3d& trans
) {
    using namespace pinocchio;

    return log6(
        m_data->oMf[m_model->getFrameId(frame)].actInv(SE3(rot, trans))
    ).toVector();
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> Model::pose(
    const std::string &frame
) {
    return std::make_tuple(
        m_data->oMf[m_model->getFrameId(frame)].translation(),
        (Eigen::Quaterniond)m_data->oMf[m_model->getFrameId(frame)].rotation()
    );
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> Model::end_effector() {
    return std::make_tuple(
        m_data->oMf[m_end_effector_index].translation(),
        (Eigen::Quaterniond)m_data->oMf[m_end_effector_index].rotation()
    );
}

} // namespace FrankaRidgeback 
