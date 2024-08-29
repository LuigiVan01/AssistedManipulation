#pragma once

#include <filesystem>
#include <iostream>

#include <nlohmann/json.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "controller/energy.hpp"
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

    struct Configuration {

        /// The filename of the URDF robot definition file to instantiate the
        // model from.
        std::string filename;

        /// The name of the end effector frame.
        std::string end_effector_frame;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            filename, end_effector_frame
        )
    };

    static inline std::filesystem::path find_path() {
        return (std::filesystem::current_path() / "model/robot.urdf").string();
    }

    /**
     * @brief Create a new instance of the robot model.
     * 
     * @param configuration The model configuration.
     * @return A pointer to the model on success, or nullptr on failure.
     */
    static std::unique_ptr<Model> create(const Configuration &configuration);

    /**
     * @brief Update the state of the model.
     * 
     * Updates the model with current joint positions and velocities.
     * 
     * @param state The joint parameters of the model.
     */
    void set(const State &state);

    /**
     * @brief Get the underlying pinocchio model.
     * @returns The underlying pinocchio model.
     */
    pinocchio::Model *get_model() const {
        return m_model.get();
    }

    /**
     * @brief Get the underlying pinocchio data.
     * 
     * See https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/topic/doc-v2/doxygen-html/structse3_1_1Data.html
     * for all available model data.
     * 
     * @return const pinocchio::Data* 
     */
    pinocchio::Data *get_data() const {
        return m_data.get();
    }

    /**
     * @brief Get the offset in the world space between two frames.
     * 
     * @param from_frame The first frame.
     * @param to_frame The second frame.
     * 
     * @returns The offset between the frames.
     */
    inline Eigen::Vector3d get_offset(
        const std::string &from_frame,
        const std::string &to_frame
    ){
        return m_data->oMf[m_model->getFrameId(to_frame)].translation() -
        m_data->oMf[m_model->getFrameId(from_frame)].translation();
    }

    /**
     * @brief Get the error screw error between two frames.
     * 
     * @param from_frame The first frame.
     * @param to_frame The second frame.
     * 
     * @returns The offset and orientation difference between the frames.
     */
    inline Eigen::Matrix<double, 6, 1> get_error(
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

    /**
     * @brief Get the pose of a frame.
     * 
     * @param frame 
     * @return std::tuple<Eigen::Vector3d, Eigen::Quaterniond> 
     */
    inline std::tuple<Eigen::Vector3d, Eigen::Quaterniond> get_pose(
        const std::string &frame
    ){
        return std::make_tuple(
            m_data->oMf[m_model->getFrameId(frame)].translation(),
            (Eigen::Quaterniond)m_data->oMf[m_model->getFrameId(frame)].rotation()
        );
    }

    /**
     * @brief Get the end effector pose.
     */
    inline std::tuple<Eigen::Vector3d, Eigen::Quaterniond> get_end_effector_pose()
    {
        return std::make_tuple(
            m_data->oMf[m_end_effector_index].translation(),
            (Eigen::Quaterniond)m_data->oMf[m_end_effector_index].rotation()
        );
    }

    /**
     * @brief Get the velocity of the end effector in the world frame.
     */
    inline Eigen::Vector3d get_end_effector_velocity() {
        return m_end_effector_velocity;
    }

    // inline Eigen::VectorXd get_joint_torques_from_external_force() {
    //     return m_joint_torques;
    // }

    /**
     * @brief Get the end effector jacobian, if calculation is enabled.
     * 
     * Jacobian is in the world frame.
     */
    inline Eigen::Ref<Eigen::Matrix<double, 6, DoF::JOINTS>> get_end_effector_jacobian() {
        return m_end_effector_jacobian;
    }

    /**
     * @brief Make a copy of the model.
     */
    inline std::unique_ptr<Model> copy()
    {
        auto model = std::make_unique<pinocchio::Model>(*m_model);
        auto data = std::make_unique<pinocchio::Data>(*model);
        return std::unique_ptr<Model>(new Model(
            m_configuration,
            std::move(model),
            std::move(data),
            m_end_effector_index
        ));
    }

private:

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
        const Configuration &configuration,
        std::unique_ptr<pinocchio::Model> model,
        std::unique_ptr<pinocchio::Data> data,
        std::size_t end_effector_index
    );

    /// The configuration of the model.
    Configuration m_configuration;

    /// The robot model.
    std::unique_ptr<pinocchio::Model> m_model;

    /// Data used for model calculations.
    std::unique_ptr<pinocchio::Data> m_data;

    /// Index of the end effector frame in the frame vector.
    std::size_t m_end_effector_index;

    /// End effector jacobian in the world frame if enabled.
    Eigen::MatrixXd m_end_effector_jacobian;

    /// The velocity of the end effector if enabled.
    Eigen::VectorXd m_end_effector_velocity;

    // Eigen::VectorXd m_joint_torques;
};

} // namespace FrankaRidgeback
