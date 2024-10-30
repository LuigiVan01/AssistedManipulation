#pragma once

#include "controller/json.hpp"
#include "controller/mppi.hpp"
#include "controller/cost.hpp"
#include "frankaridgeback/dynamics.hpp"

namespace FrankaRidgeback {

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

        /// Avoid colliding with itself.
        bool enable_self_collision_avoidance;

        /// If the objective should limit power usage.
        bool enable_power_limit;

        /// If the objective should have minimum reach.
        bool enable_reach_limits;

        /// Lower joint limits if enabled.
        std::array<LeftInverseBarrierFunction, DoF::JOINTS> lower_joint_limit;

        /// Upper joint limits if enabled.
        std::array<RightInverseBarrierFunction, DoF::JOINTS> upper_joint_limit;

        /// Self collision cost.
        LeftInverseBarrierFunction self_collision_limit;

        std::array<double, 8> self_collision_radii;

        /// Maximum reach if enabled.
        RightInverseBarrierFunction maximum_reach_limit;

        // JSON conversion for track point objective configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            point, enable_joint_limits, enable_self_collision_avoidance,
            enable_power_limit, enable_reach_limits, lower_joint_limit,
            upper_joint_limit, self_collision_limit, self_collision_radii,
            maximum_reach_limit
        )
    };

    /**
     * @brief Pairs of links to be checked for self collision.
     */
    // static inline std::vector<std::tuple<std::string, std::string>>
    // SELF_COLLISION_LINKS = {{
    //     {Frame::ARM_MOUNT_JOINT, Frame::PANDA_JOINT1},
    //     {Frame::PANDA_JOINT1, Frame::PANDA_JOINT2},
    //     {Frame::PANDA_JOINT2, Frame::PANDA_JOINT3},
    //     {Frame::PANDA_JOINT3, Frame::PANDA_JOINT4},
    //     {Frame::PANDA_JOINT4, Frame::PANDA_JOINT5},
    //     {Frame::PANDA_JOINT5, Frame::PANDA_JOINT6},
    //     {Frame::PANDA_JOINT7, Frame::ARM_MOUNT_JOINT},
    // }};

    /**
     * @brief The default configuration of the track point objective.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .point = Vector3d(1.0, 1.0, 1.0),
        .enable_joint_limits = true,
        .enable_self_collision_avoidance = false,
        .enable_power_limit = false,
        .enable_reach_limits = false,
        .lower_joint_limit = {{
            {-2.0,    1.0}, // Base x
            {-2.0,    0.0}, // Base y
            {-6.28,   0.0}, // Base yaw
            {-2.8,    10.0}, // Joint1
            {-1.745,  50.0}, // Joint2
            {-2.8,    10.0}, // Joint3
            {-3.0718, 10.0}, // Joint4
            {-2.7925, 10.0}, // Joint5
            {0.349,   10.0}, // Joint6
            {-2.967,  10.0}, // Joint7
            {0.0,     10.0}, // Gripper x
            {0.0,     10.0}  // Gripper y
        }},
        .upper_joint_limit = {{
            {2.0,     0.0}, // Base x
            {2.0,     0.0}, // Base y
            {6.28,    0.0}, // Base yaw
            {2.8,     10.0}, // Joint1
            {1.745,   50.0}, // Joint2
            {2.8,     10.0}, // Joint3
            {0.0,     10.0}, // Joint4
            {2.7925,  10.0}, // Joint5
            {4.53785, 10.0}, // Joint6
            {2.967,   10.0}, // Joint7
            {0.5,     10.0}, // Gripper x
            {0.5,     10.0}  // Gripper y
        }},
        .self_collision_limit = {0.0, 1.0},
        .self_collision_radii = {0.75, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
        .maximum_reach_limit = {0.8, 1.0}
    };

    /**
     * @brief Get the number of state degrees of freedom.
     */
    inline constexpr int get_state_dof() override {
        return DoF::STATE;
    }

    /**
     * @brief Get the number of control degrees of freedom.
     */
    inline constexpr int get_control_dof() override {
        return DoF::CONTROL;
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

    /**
     * @brief Unused.
     */
    void reset(double) override {};

    /**
     * @brief Make a copy of the objective function.
     * @returns The copy of the objective function.
     */
    inline std::unique_ptr<mppi::Cost> copy() override {
        return std::make_unique<TrackPoint>(*this);
    }

private:

    /**
     * @brief Initialise the track point objective function.
     * @param configuration The configuration of the track point objective. 
     */
    inline TrackPoint(const Configuration &configuration)
        : m_configuration(configuration)
    {}

    /**
     * @brief Get the cost of the euclean distance from the end effector
     * position to the tracked point.
     * 
     * @param dynamics Pointer to the dynamics.
     * @returns The cost.
     */
    double point_cost(Dynamics *dynamics);

    /**
     * @brief Get the cost of the joint limit proximity.
     * 
     * @param state The state of the franka-ridgeback.
     * @return The cost.
     */
    double joint_limit_cost(const State &state);

    /**
     * @brief Get the cost of self collision.
     * 
     * Self collision is implemented using imaginary spheres centered on each
     * joint. The joints are considered colliding if the spheres are
     * intersecting. The radii of the spheres can be adjusted to change the
     * sensitivity to self collision.
     * 
     * @param dynamics Pointer to the dynamics.
     * @returns The cost.
     */
    double self_collision_cost(Dynamics *dynamics);

    /**
     * @brief Get the cost of the current power usage.
     * 
     * @param dynamics Pointer to the dynamics.
     * @returns The cost.
     */
    double power_cost(Dynamics *dynamics);

    /**
     * @brief Get the cost of minimum reach.
     *
     * @param dynamics pointer to the dynamics.
     * @returns The cost.
     */
    double reach_cost(Dynamics *dynamics);

    /// The configuration of the track point objective, including point to
    /// track.
    Configuration m_configuration;
};

} // namespace FrankaRidgeback
