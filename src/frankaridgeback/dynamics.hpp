#pragma once

#include <utility>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "controller/energy.hpp"
#include "controller/json.hpp"
#include "controller/forecast.hpp"
#include "controller/mppi.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

/**
 * @brief The dynamics of the model parameters.
 * 
 * Defines how State evolves with Control input.
 */
class Dynamics : public mppi::Dynamics
{
public:

    struct Configuration {

        /// The filename of the URDF robot definition file to instantiate the
        // model from.
        std::string filename;

        /// The name of the end effector frame.
        std::string end_effector_frame;

        /// The initial energy in the energy tank.
        double energy;

        /// JSON conversion for dynamics configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            filename, end_effector_frame, energy
        );
    };

    static inline std::filesystem::path find_path() {
        return (std::filesystem::current_path() / "model/robot.urdf").string();
    }

    /**
     * @brief Create a new franka ridgeback dynamics object.
     * 
     * @param configuration The configuration of the dynamics.
     * @param force_predictor An optional force estimation strategy.
     * 
     * @returns A pointer to the dynamics on success or nullptr on failure.
     */
    static std::unique_ptr<Dynamics> create(
        const Configuration &configuration,
        Forecast *force_predictor = nullptr
    );

    /**
     * @brief Get the number of degrees of freedom for the frankaridgeback
     * state.
     */
    inline constexpr int state_dof() override {
        return DoF::STATE;
    }

    /**
     * @brief Get the number of degrees of freedom for the frankaridgeback
     * control.
     */
    inline constexpr int control_dof() override {
        return DoF::CONTROL;
    }

    /**
     * @brief Get the dynamics state.
     */
    inline Eigen::Ref<Eigen::VectorXd> get() {
        return m_state;
    }

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

    /**
     * @brief Get the end effector jacobian, if calculation is enabled.
     * 
     * Jacobian is in the world frame.
     */
    inline Eigen::Ref<Eigen::Matrix<double, 6, DoF::JOINTS>> get_end_effector_jacobian() {
        return m_end_effector_jacobian;
    }

    /**
     * @brief Set the dynamics simulation to a given state.
     * @param state The system state.
     * @param time The time of the dynamincs state.
     */
    void set(const Eigen::VectorXd &state, double time) override;

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) override;

    /**
     * @brief Copy the dynamics.
     */
    std::unique_ptr<mppi::Dynamics> copy() override;

private:

    /**
     * @brief Calculates kinematic information after position and velocity
     * updates.
     */
    void update_kinematics();

    /**
     * @brief Initialise the dynamics parameters.
     * 
     * @param model Pointer to the pinnochio model.
     * @param data Pointer to the pinnochio data.
     * @param end_effector_index Index into the vector of model joint frames for
     * the end effector.
     * @param force_predictor An optional force estimation strategy.
     * @param energy The initial energy in the energy tank.
     */
    Dynamics(
        std::unique_ptr<pinocchio::Model> &&model,
        std::unique_ptr<pinocchio::Data> &&data,
        std::unique_ptr<Forecast::Handle> &&force_predictor_handle,
        std::size_t end_effector_index,
        double energy
    );

    /// The robot model.
    std::unique_ptr<pinocchio::Model> m_model;

    /// Data used for model calculations.
    std::unique_ptr<pinocchio::Data> m_data;

    /// Optional pointer to the force predictor.
    std::unique_ptr<Forecast::Handle> m_wrench_forecast;

    /// The current joint positions.
    Eigen::Vector<double, DoF::JOINTS> m_position;

    /// The current joint velocities.
    Eigen::Vector<double, DoF::JOINTS> m_velocity;

    /// The current torques applied by the controller to the joints.
    Eigen::Vector<double, DoF::JOINTS> m_torque;

    /// The acceleration of the joints from forward dynamics.
    Eigen::Vector<double, DoF::JOINTS> m_acceleration;

    /// Index of the end effector frame in the frame vector.
    std::size_t m_end_effector_index;

    /// End effector jacobian in the world frame if enabled.
    Eigen::MatrixXd m_end_effector_jacobian;

    /// The velocity of the end effector if enabled.
    Eigen::VectorXd m_end_effector_velocity;

    /// The energy tank.
    EnergyTank m_energy_tank;

    /// The state to yield to the mppi trajectory generator.
    State m_state;

    /// Time of the last dynamics set.
    double m_time;
};

} // namespace FrankaRidgeback
