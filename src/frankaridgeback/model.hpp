#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

/**
 * @brief The robot model.
 * 
 * This class can be updated with parameters and queried for the current robot
 * state.
 * 
 * This class uses pinocchio for calculating robot kinematics and dynamics.
 */
class Model
{
public:

    /**
     * @brief Create a new instance of the robot model.
     * 
     * @param filename The filename of the URDF robot definition file to
     * instantiate the model from.
     * @param end_effector_frame The name of the end effector frame.
     * 
     * @return A pointer to the model on success, or nullptr on failure.
     */
    static std::unique_ptr<Model> create(
        const std::string &filename,
        const std::string &end_effector_frame = "panda_grasp"
    );

    /**
     * @brief Update the state of the model.
     * @param state The joint parameters of the model.
     */
    void set(const State &state);

    // void Model::set(
    //     const State &state,
    //     const Velocity &velocity
    // ) {
    //     pinocchio::forwardKinematics(m_model, m_data, state, velocity);
    //     pinocchio::updateFramePlacements(m_model, m_data);
    // }

    /**
     * @brief Update the state of the model and the joint velocities.
     * 
     * @param state The joint parameters of the model.
     * @param velocity The velocities of each joint.
     */
    // void update(const State &state, const Velocity &velocity);

    Eigen::Vector3d offset(
        const std::string &from_frame,
        const std::string &to_frame
    );

    Eigen::Matrix<double, 6, 1> error(
        const std::string &from_frame,
        const std::string &to_frame
    );

    Eigen::Matrix<double, 6, 1> error(
        const std::string &frame,
        const Eigen::Quaterniond& rot,
        const Eigen::Vector3d& trans
    );

    std::tuple<Eigen::Vector3d, Eigen::Quaterniond> pose(
        const std::string &frame
    );

    std::tuple<Eigen::Vector3d, Eigen::Quaterniond> end_effector();

    inline std::unique_ptr<Model> copy()
    {
        auto model = std::make_unique<pinocchio::Model>(*m_model);
        auto data = std::make_unique<pinocchio::Data>(*model);
        return std::unique_ptr<Model>(new Model(
            std::move(model),
            std::move(data),
            m_end_effector_index
        ));
    }

private:

    Model() = default;

    /**
     * @brief Initialise the dynamics model.
     * 
     * To create an instance, use Model::create().
     * 
     * @param model Pointer to the pinnochio model.
     * @param data Pointer to the pinnochio data.
     * @param end_effector_index Index into the vector of joint frames for the
     * end effector.
     */
    Model(
        std::unique_ptr<pinocchio::Model> model,
        std::unique_ptr<pinocchio::Data> data,
        std::size_t end_effector_index
    );

    /// The robot model.
    std::unique_ptr<pinocchio::Model> m_model;

    /// Data used for model calculations.
    std::unique_ptr<pinocchio::Data> m_data;

    /// Index of the end effector frame in the frame vector.
    std::size_t m_end_effector_index;
};

} // namespace FrankaRidgeback
