#pragma once

#include "controller/json.hpp"
#include "controller/mppi.hpp"
#include "controller/cost.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/dynamics.hpp"
#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

/**
 * @brief Objective function of the franka research 3 ridgeback assisted
 * manipulation task.
 */
class AssistedManipulation : public mppi::Cost
{
public:

    struct Configuration {

        /// If joint limit costs are enabled.
        bool enable_joint_limit = true;

        /// If self collision costs are enabled.
        bool enable_self_collision = true;

        /// If tracking the expected trajectory is enabled.
        bool enable_trajectory_tracking = true;

        /// If reach costs are enabled.
        bool enable_reach_limit = false;

        /// If end effector manipulability is maximised.
        bool enable_maximise_manipulability = false;

        /// If the power used by the trajectory is minimised.
        bool enable_maximum_power = false;

        /// If variable damping should be enabled.
        bool enable_variable_damping = false;

        /// Lower joint limits if enabled.
        std::array<QuadraticCost, DoF::JOINTS> lower_joint_limit;

        /// Upper joint limits if enabled.
        std::array<QuadraticCost, DoF::JOINTS> upper_joint_limit;

        /// Self collision cost.
        QuadraticCost self_collision;

        /// Minimum reach if enabled.
        QuadraticCost minimum_reach;

        /// Maximum reach if enabled.
        QuadraticCost maximum_reach;

        /// 
        QuadraticCost trajectory;

        /// Manipulability limits if enabled.
        QuadraticCost minimum_manipulability;

        /// Maximum power (joules per second) usage if enabled. 
        QuadraticCost maximum_power;

        /// The maximum damping that occurs when the end effector has zero
        /// velocity. The A in c(v) = Ae^{lambda * v}
        double variable_damping_maximum;

        /// The exponential drop-off from variable_damping_maximum with respect
        /// to velocity. The lambda in c(v) = Ae^{lambda * v}
        double variable_damping_dropoff;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            enable_joint_limit, enable_reach_limit, 
            enable_maximise_manipulability, enable_maximum_power,
            enable_variable_damping, lower_joint_limit, upper_joint_limit,
            self_collision, maximum_reach, minimum_reach,
            minimum_manipulability, maximum_power, variable_damping_maximum,
            variable_damping_dropoff
        )
    };

    /**
     * @brief Pairs of links to be checked for self collision.
     */
    static inline std::vector<std::tuple<std::string, std::string>>
    SELF_COLLISION_LINKS = {{
        {Frame::PANDA_JOINT_FRANKA_MOUNT_LINK, Frame::PANDA_JOINT1},
        {Frame::PANDA_JOINT1, Frame::PANDA_JOINT2},
        {Frame::PANDA_JOINT2, Frame::PANDA_JOINT3},
        {Frame::PANDA_JOINT3, Frame::PANDA_JOINT4},
        {Frame::PANDA_JOINT4, Frame::PANDA_JOINT5},
        {Frame::PANDA_JOINT5, Frame::PANDA_JOINT6},
        {Frame::PANDA_JOINT7, Frame::PANDA_JOINT_FRANKA_MOUNT_LINK},
    }};

    /**
     * @brief The default configuration of the assisted manipulation objective
     * 
     * The fidelity of the joint limits probably doesn't need to be this high.
     * 
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .enable_joint_limit = true,
        .enable_self_collision = false,
        .enable_trajectory_tracking = true,
        .enable_reach_limit = false,
        .enable_maximise_manipulability = false,
        .enable_maximum_power = false,
        .enable_variable_damping = false,
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
        return DoF::STATE;
    }

    /**
     * @brief Get the number of control degrees of freedom.
     */
    inline constexpr int get_control_dof() override {
        return DoF::CONTROL;
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
     * @param dynamics Pointer to the dynamics at the time step.
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
     * @brief Reset the objective function cost.
     */
    void reset() override {};

    /**
     * @brief Make a copy of the objective function.
     */
    inline std::unique_ptr<mppi::Cost> copy() override {
        return std::unique_ptr<AssistedManipulation>(
            new AssistedManipulation(m_configuration)
        );
    }

private:

    /**
     * @brief Initialise the assisted manipulation cost.
     * 
     * @param model Pointer to the robot model.
     * @param configuration The configuration of the objective function.
     */
    AssistedManipulation(const Configuration &configuration);

    /**
     * @brief Penalises joint positions that exceed their limits.
     * 
     * @param state The current state of the frankaridgeback.
     * @returns A cost depending on the joint state.
     */
    double joint_limit_cost(const State &state);

    /**
     * @brief Penalise configurations that.
     * 
     * This is an approximation function where each link has an imagined sphere
     * located, with some defined radius that is smaller than the maximum link
     * dimension (otherwise will always be in collision with adjacent links).
     * The joints are considered colliding if the spheres are intersecting. The
     * radii of the spheres can be adjusted to change the sensitivity to self 
     * collision.
     * 
     * @param dynamcs The current dynamics state.
     * @returns A cost penalising self collision.
     */
    double self_collision_cost(Dynamics *dynamics);

    /**
     * @brief Reward trajectories that tend towards the expected trajectory
     * given the end effector force.
     * 
     * @param dynamcs The current dynamics state.
     * @param time The time of the dynamics.
     * @returns A cost rewarding trajectories towards.
     */
    double trajectory_cost(Dynamics *dynamics, double time);

    /**
     * @brief Penalises end effector positions that are too close or too far
     * from the robot base.
     * 
     * Implemented as a check on the euclidean distance between the base of the
     * arm and the end effector frame.
     * 
     * @param dynamics The current dynamics state.
     * @returns A cost penalising end effector positions too close or too far
     * from the base.
     */
    double reach_cost(Dynamics *dynamics);

    /**
     * @brief Penalises dynamics states that exceed the power limit.
     * 
     * @param dynamics The current dynamics state.
     * @returns A cost penalising dynamics that go over the maximum power draw.
     */
    double power_cost(Dynamics *dynamics);

    /**
     * @brief Rewards higher manipulability joint configurations.
     * 
     * Manipulability is calculated based on the end effector jacobian and
     * resulting manipulability ellipsoid.
     * 
     * Cost metric is proportional to `sqrt(det(J * J^T))` that is itself
     * proportional to the volume of the manipulability ellipsoid, clipped above
     * 1e-10. Jacobian in spatial frame. Greater values are better.
     * 
     * @param dynamics The current dynamics state.
     * @returns A cost rewarding manipulability.
     */
    double manipulability_cost(Dynamics *dynamics);

    double variable_damping_cost(const State &state);

    /// The configuration of the objective function.
    Configuration m_configuration;

    /// Spatial jacobian.
    Eigen::Matrix<double, 6, 6> m_space_jacobian;

    double m_joint_cost;

    double m_self_collision_cost;

    double m_trajectory_cost;

    double m_reach_cost;

    double m_power_cost;

    double m_manipulability_cost;

    double m_variable_damping_cost;

    double m_cost;
};

} // namespace FrankaRidgeback
