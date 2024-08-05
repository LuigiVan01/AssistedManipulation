#pragma once

#include "controller/mppi.hpp"
#include "frankaridgeback/model.hpp"

/**
 * @brief Objective function of the franka research 3 ridgeback assisted
 * manipulation task.
 */
class AssistedManipulation : public mppi::Cost
{
public:

    /// The default lower joint limits.
    inline static Eigen::Vector<double, FrankaRidgeback::DoF::JOINTS>
    s_default_lower_joint_limits {
        -2.0, -2.0, -6.28,
        -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
        0.5, 0.5
    };

    /// The default upper joint limits.
    inline static Eigen::Vector<double, FrankaRidgeback::DoF::JOINTS>
    s_default_upper_joint_limits {
        2.0, 2.0, 6.28,
        2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973,
        0.5, 0.5
    };

    struct Configuration {

        /// The configuration of the model.
        FrankaRidgeback::Model::Configuration model;

        /// Penalise approaching joint limits.
        bool enable_joint_limits;

        /// The lower joint limits.
        Eigen::VectorXd lower_joint_limit;

        /// The upper joint limits.
        Eigen::VectorXd upper_joint_limit;
    };

    /**
     * @brief Get the number of state degrees of freedom.
     */
    inline constexpr int state_dof() override {
        return FrankaRidgeback::DoF::STATE;
    }

    /**
     * @brief Get the number of control degrees of freedom.
     */
    inline constexpr int control_dof() override {
        return FrankaRidgeback::DoF::CONTROL;
    }

    /**
     * @brief Create an assisted manipulation objective function.
     * 
     * @param configuration The configuration of the objective function.
     * @returns A pointer to the objective on success, or nullptr on failure.
     */
    static std::unique_ptr<AssistedManipulation> create(
        Configuration &&configuration
    );

    /**
     * @brief Get the cost of a state and control input over dt.
     * 
     * @param state The state of the system.
     * @param control The control parameters applied to the state.
     * @param time The current time.
     * 
     * @returns The cost of the step.
     */
    double get(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        double time
    ) override;

    /**
     * @brief Reset the objective function cost.
     */
    void reset() override {};

    /**
     * @brief Get the franka ridgeback model.
     */
    std::unique_ptr<FrankaRidgeback::Model> &model() {
        return m_model;
    }

    /**
     * @brief Make a copy of the objective function.
     */
    inline std::unique_ptr<mppi::Cost> copy() override {
        return std::unique_ptr<AssistedManipulation>(
            new AssistedManipulation(
                std::move(m_model->copy()),
                m_configuration
            )
        );
    }

private:

    /**
     * @brief Initialise the assisted manipulation cost.
     * 
     * @param model Pointer to the robot model.
     * @param configuration The configuration of the objective function.
     */
    AssistedManipulation(
    std::unique_ptr<FrankaRidgeback::Model> &&model,
    const Configuration &configuration
  );

    /// The configuration of the objective function.
    Configuration m_configuration;

    /// Pointer to the model to calculate proximity to the point.
    std::unique_ptr<FrankaRidgeback::Model> m_model;
};
