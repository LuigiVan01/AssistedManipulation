#pragma once

#include <filesystem>

#include <nlohmann/json.hpp>

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

    /**
     * @brief Get the end effector position and orientation.
     */
    std::tuple<Eigen::Vector3d, Eigen::Quaterniond> end_effector_pose();

    /**
     * @brief Get the velocity of the end effector in the world frame.
     */
    Eigen::Vector3d end_effector_velocity();

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
