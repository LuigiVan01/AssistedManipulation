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

        /**
         * @brief A generic objective function for evaluating a single variable.
         */
        struct Limit {

            /// The limits to apply.
            double limit;

            /// Constant cost added when limit is breached.
            double constant_cost = 1'000;

            /// Additional cost added proportional to how much the limit is
            /// breached.
            double proportional_cost = 100'000;
        };

        /// The configuration of the model.
        FrankaRidgeback::Model::Configuration model;

        /// If joint limit costs are enabled.
        bool enable_joint_limit = true;

        /// If reach costs are enabled.
        bool enable_reach_limit = true;

        /// If end effector manipulability is maximised.
        bool enable_maximise_manipulability = true;

        /// If the power used by the trajectory is minimised.
        bool enable_minimise_power = true;

        /// If variable damping should be enabled.
        bool enable_variable_damping = true;

        /// Lower joint limits if enabled.
        std::vector<Limit> lower_joint_limit;

        /// Upper joint limits if enabled.
        std::vector<Limit> upper_joint_limit;

        /// Maximum reach if enabled.
        Limit maximum_reach;

        /// Minimum reach if enabled.
        Limit minimum_reach;

        /// Manipulability limits if enabled. Relative to `sqrt(det(J * J^T))`
        /// that is proportional to the volume of the manipulability ellipsoid,
        /// clipped above 1e-10. Jacobian in spatial frame. Greater values are
        /// better. Limit is a lower bound on this value.
        Limit minimum_manipulability;

        /// Maximum power (joules per second) usage if enabled. 
        Limit maximum_power;

        double variable_damping_maximum;

        double variable_damping_dropoff;
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
        const Configuration &configuration
    );

    /**
     * @brief Get the cumulative power cost.
     */
    inline double get_power_cost() {
        return m_power_cost;
    }

    /**
     * 
    */
    inline double get_manipulability_cost() {
        return m_manipulability_cost;
    }

    /**
     * 
    */
    inline double get_joint_cost() {
        return m_joint_cost;
    }

    /**
     * 
    */
    inline double get_reach_cost() {
        return m_reach_cost;
    }

    /**
     * 
    */
    inline double get_variable_damping_cost() {
        return m_variable_damping_cost;
    }

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

    double power_cost(
        const FrankaRidgeback::State &state,
        const FrankaRidgeback::Control &control
    );

    double manipulability_cost();

    double joint_limit_cost(const FrankaRidgeback::State &state);

    double reach_cost();

    double variable_damping_cost(const FrankaRidgeback::State &state);

    /// The configuration of the objective function.
    Configuration m_configuration;

    /// Pointer to the model to calculate proximity to the point.
    std::unique_ptr<FrankaRidgeback::Model> m_model;

    /// Spatial jacobian.
    Eigen::MatrixXd m_space_jacobian;

    double m_cost;

    double m_power_cost;

    double m_manipulability_cost;

    double m_joint_cost;

    double m_reach_cost;

    double m_variable_damping_cost;
};
