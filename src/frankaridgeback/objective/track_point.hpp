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
        bool enable_joint_limits = true;

        /// If the objective should minimise power usage.
        bool enable_minimuse_power = true;

        /// Lower joint limits if enabled.
        std::array<QuadraticCost, FrankaRidgeback::DoF::JOINTS> lower_joint_limit;

        /// Upper joint limits if enabled.
        std::array<QuadraticCost, FrankaRidgeback::DoF::JOINTS> upper_joint_limit;

        /// Maximum power usage if enabled. 
        QuadraticCost maximum_power;

        // JSON conversion for track point objective configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, point)
    };

    inline constexpr int get_state_dof() override {
        return FrankaRidgeback::DoF::STATE;
    }

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
