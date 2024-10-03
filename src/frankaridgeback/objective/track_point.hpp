#pragma once

#include "controller/json.hpp"
#include "controller/mppi.hpp"
#include "controller/cost.hpp"
#include "frankaridgeback/dynamics.hpp"

/**
 * @brief Cost function of the panda research 3 ridgeback assisted manipulation
 * task.
 */
class TrackPoint : public mppi::Cost
{
public:

    struct Configuration {

        /// The position in 3D space to track.
        Eigen::Vector3d point;

        /// If the objective should avoid joint limits.
        bool enable_joint_limits;

        /// If the objective should limit power usage.
        bool enable_power_limit;

        /// Lower joint limits if enabled.
        std::array<QuadraticCost, FrankaRidgeback::DoF::JOINTS> lower_joint_limit;

        /// Upper joint limits if enabled.
        std::array<QuadraticCost, FrankaRidgeback::DoF::JOINTS> upper_joint_limit;

        /// Maximum power usage if enabled. 
        QuadraticCost maximum_power;

        // JSON conversion for track point objective configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, point)
    };

    /**
     * @brief The default configuration of the track point objective.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .point = Vector3d(1.0, 1.0, 1.0),
        .enable_joint_limits = true,
        .enable_power_limit = false,
        .lower_joint_limit = {{
            {-2.0,    1'000, 100'00}, // Base rotation
            {-2.0,    1'000, 100'00}, // Base x
            {-6.28,   1'000, 100'00}, // Base y
            {-2.8973, 1'000, 100'00}, // Joint1
            {-1.7628, 1'000, 100'00}, // Joint2
            {-2.8973, 1'000, 100'00}, // Joint3
            {-3.0718, 1'000, 100'00}, // Joint4
            {-2.8973, 1'000, 100'00}, // Joint5
            {-0.0175, 1'000, 100'00}, // Joint6
            {-2.8973, 1'000, 100'00}, // Joint7
            {0.5,     1'000, 100'00}, // Gripper x
            {0.5,     1'000, 100'00}  // Gripper y
        }},
        .upper_joint_limit = {{
            {2.0,    1'000, 100'00}, // Base rotation
            {2.0,    1'000, 100'00}, // Base x
            {6.28,   1'000, 100'00}, // Base y
            {2.8973, 1'000, 100'00}, // Joint1
            {1.7628, 1'000, 100'00}, // Joint2
            {2.8973, 1'000, 100'00}, // Joint3
            {3.0718, 1'000, 100'00}, // Joint4
            {2.8973, 1'000, 100'00}, // Joint5
            {0.0175, 1'000, 100'00}, // Joint6
            {2.8973, 1'000, 100'00}, // Joint7
            {0.5,    1'000, 100'00}, // Gripper x
            {0.5,    1'000, 100'00}  // Gripper y
        }}
    };

    /**
     * @brief Get the number of state degrees of freedom.
     */
    inline constexpr int get_state_dof() override {
        return FrankaRidgeback::DoF::STATE;
    }

    /**
     * @brief Get the number of control degrees of freedom.
     */
    inline constexpr int get_control_dof() override {
        return FrankaRidgeback::DoF::CONTROL;
    }

    /**
     * @brief Create a track point objective function.
     * 
     * @param configuration The configuration of the objective function.
     * @return A pointer to the cost instance on success, or nullptr on failure.
     */
    static inline std::unique_ptr<TrackPoint> create(
        const Configuration &configuration
    ) {
        return std::unique_ptr<TrackPoint>(
            new TrackPoint(configuration)
        );
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
    double get_cost(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        mppi::Dynamics *dynamics,
        double time
    ) override;

    void reset() override {};

    inline std::unique_ptr<mppi::Cost> copy() override {
        return std::make_unique<TrackPoint>(*this);
    }

private:

    /**
     * @brief Initialise the track point objective function.
     * @param configuration The configuration of the track point objective. 
     */
    inline TrackPoint(const Configuration &configuration)
        : m_configuration(configuration.point)
    {}

    /**
     * @brief Get the cost of the euclean distance from the end effector
     * position to the tracked point.
     * 
     * @param dynamics Pointer to the dynamics.
     * @returns The cost.
     */
    double point_cost(FrankaRidgeback::Dynamics *dynamics);

    /**
     * @brief Get the cost of the joint limit proximity.
     * 
     * @param state The state of the franka-ridgeback.
     * @return The cost.
     */
    double joint_cost(const FrankaRidgeback::State &state);

    /**
     * @brief Get the cost of the current power usage.
     * 
     * @param dynamics Pointer to the dynamics.
     * @returns The cost.
     */
    double power_cost(FrankaRidgeback::Dynamics *dynamics);

    /// The configuration of the track point objective, including point to
    /// track.
    Configuration m_configuration;
};
