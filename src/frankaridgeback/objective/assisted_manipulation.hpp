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
        bool enable_joint_limit;

        /// If self collision costs are enabled.
        bool enable_self_collision_limit;

        /// If workspace costs are enabled.
        bool enable_workspace_limit;

        /// If energy tank should be enabled.
        bool enable_energy_limit;

        /// If the joint velocities should be minimised.
        bool enable_velocity_cost;

        /// If tracking the expected trajectory is enabled.
        bool enable_trajectory_cost;

        /// If end effector manipulability is maximised.
        bool enable_manipulability_cost;

        /// Lower joint limits if enabled.
        std::array<LeftInverseBarrierFunction, DoF::JOINTS> lower_joint_limit;

        /// Upper joint limits if enabled.
        std::array<RightInverseBarrierFunction, DoF::JOINTS> upper_joint_limit;

        /// Self collision cost.
        LeftInverseBarrierFunction self_collision_limit;

        std::array<double, 8> self_collision_radii;

        /// Trajectory not infront cost.
        LeftInverseBarrierFunction workspace_limit_above;

        LeftInverseBarrierFunction workspace_limit_infront;

        RightInverseBarrierFunction workspace_limit_reach;

        QuadraticCost workspace_cost_yaw;

        /// Maximum energy tank energy if enabled.
        LeftInverseBarrierFunction energy_limit_below;

        RightInverseBarrierFunction energy_limit_above;

        /// Velocity minimisation cost.
        std::array<QuadraticCost, DoF::JOINTS> velocity_cost;

        double trajectory_target_scale;

        double trajectory_target_maximum;

        /// Cost of matching the forecast trajectory.
        QuadraticCost trajectory_position_cost;

        double trajectory_position_threshold;

        QuadraticCost trajectory_velocity_cost;

        /// The minimum velocity for trajectory tracking.
        double trajectory_velocity_minimum;

        /// The maximum velocity for trajectory tracking.
        double trajectory_velocity_maximum;

        /// Rate of dropoff between maximum and zero velocity.
        double trajectory_velocity_dropoff;

        /// Manipulability limits if enabled.
        QuadraticCost manipulability_cost;

        double manipulability_minimum;

        // JSON conversion for assisted manipulation objective configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            enable_joint_limit,
            enable_self_collision_limit,
            enable_workspace_limit,
            enable_energy_limit,
            enable_velocity_cost,
            enable_trajectory_cost,
            enable_manipulability_cost,
            lower_joint_limit,
            upper_joint_limit,
            self_collision_limit,
            workspace_limit_above,
            workspace_limit_infront,
            workspace_limit_reach,
            workspace_cost_yaw,
            energy_limit_below,
            energy_limit_above,
            velocity_cost,
            trajectory_target_scale,
            trajectory_target_maximum,
            trajectory_position_cost,
            trajectory_position_threshold,
            trajectory_velocity_cost,
            trajectory_velocity_minimum,
            trajectory_velocity_maximum,
            trajectory_velocity_dropoff,
            manipulability_cost,
            manipulability_minimum
        )
    };

    /**
     * @brief The default configuration of the assisted manipulation objective
     * 
     * The fidelity of the joint limits probably doesn't need to be this high.
     * 
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .enable_joint_limit = true,
        .enable_self_collision_limit = true,
        .enable_workspace_limit = true,
        .enable_energy_limit = false,
        .enable_velocity_cost = true,
        .enable_trajectory_cost = true,
        .enable_manipulability_cost = true,
        .lower_joint_limit = {{
            {-2.0,    0.0}, // Base x
            {-2.0,    0.0}, // Base y
            {-6.28,   0.0}, // Base yaw
            {-2.8,    1.0}, // Joint1
            {-1.745,  1.0}, // Joint2
            {-2.8,    1.0}, // Joint3
            {-3.0718, 1.0}, // Joint4
            {-2.7925, 1.0}, // Joint5
            {0.349,   1.0}, // Joint6
            {-2.967,  1.0}, // Joint7
            {0.0,     0.0}, // Gripper x
            {0.0,     0.0}  // Gripper y
        }},
        .upper_joint_limit = {{
            {2.0,     0.0}, // Base x
            {2.0,     0.0}, // Base y
            {6.28,    0.0}, // Base yaw
            {2.8,     1.0}, // Joint1
            {1.745,   1.0}, // Joint2
            {2.8,     1.0}, // Joint3
            {0.0,     1.0}, // Joint4
            {2.7925,  1.0}, // Joint5
            {4.53785, 1.0}, // Joint6
            {2.967,   1.0}, // Joint7
            {0.5,     0.0}, // Gripper x
            {0.5,     0.0}  // Gripper y
        }},
        .self_collision_limit = {0.0, 1.0},
        .self_collision_radii = {0.75, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
        .workspace_limit_above = {0.0, 1.0},
        .workspace_limit_infront = {0.0, 1.0},
        .workspace_limit_reach = {1.0, 1.0},
        .workspace_cost_yaw = { .quadratic_cost = 250 },
        .energy_limit_below = {0.0, 10.0},
        .energy_limit_above = {20.0, 10.0},
        .velocity_cost = {{
            {0.0, 0.0, 1000.0}, // Base x
            {0.0, 0.0, 1000.0}, // Base y
            {0.0, 0.0, 100.0}, // Base yaw
            {0.0, 0.0, 10.0}, // Joint1
            {0.0, 0.0, 10.0}, // Joint2
            {0.0, 0.0, 10.0}, // Joint3
            {0.0, 0.0, 10.0}, // Joint4
            {0.0, 0.0, 10.0}, // Joint5
            {0.0, 0.0, 10.0}, // Joint6
            {0.0, 0.0, 10.0}, // Joint7
            {0.0, 0.0, 0.0}, // Gripper x5
            {0.0, 0.0, 0.0}  // Gripper y
        }},
        .trajectory_target_scale = 1e-2,
        .trajectory_target_maximum = 1.0,
        .trajectory_position_cost = {
            .constant_cost = 100,
            .quadratic_cost = 500.0
        },
        .trajectory_position_threshold = 0.05,
        .trajectory_velocity_cost = {
            .constant_cost = 0,
            .quadratic_cost = 500.0
        },
        .trajectory_velocity_minimum = 0.05,
        .trajectory_velocity_maximum = 1.0,
        .trajectory_velocity_dropoff = 0.7,
        .manipulability_cost = { .quadratic_cost = 10 },
        .manipulability_minimum = 0.0
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

    inline double get_joint_limit_cost() const {
        return m_joint_cost;
    }

    inline double get_self_collision_cost() const {
        return m_self_collision_cost;
    }

    inline double get_workspace_cost() const {
        return m_workspace_cost;
    }

    inline double get_energy_tank_cost() const {
        return m_energy_cost;
    }

    inline double get_joint_velocity_cost() const {
        return m_velocity_cost;
    }

    inline double get_trajectory_cost() const {
        return m_trajectory_cost;
    }

    inline double get_manipulability_cost() const {
        return m_manipulability_cost;
    }

    /**
     * @brief Reset the objective function cost.
     * @param time The initial objective time.
     */
    void reset(double time) override;

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
     * @brief Penalises end effector positions that are too close or too far
     * from the robot base.
     * 
     * Implemented as a check on the euclidean distance between the base of the
     * arm and the end effector frame.
     * 
     * @param dynamics The current dynamics state.
     * @param target The target position.
     * @returns A cost penalising end effector positions too close or too far
     * from the base.
     */
    double workspace_cost(Dynamics *dynamics);

    /**
     * @brief Penalises trajectories that deplete the energy tank and maximum
     * tank energy.
     * 
     * @param dynamics Pointer to the dynamics object.
     * @returns The cost. 
     */
    double energy_cost(Dynamics *dynamics);

    double velocity_cost(const State &state);

    /**
     * @brief Reward trajectories that tend towards the expected trajectory
     * given the end effector force.
     * 
     * @param dynamcs The current dynamics state.
     * @param time The time of the dynamics.
     * @returns A cost rewarding trajectories towards.
     */
    double trajectory_cost(const Dynamics *dynamics, double time);

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

    /// The configuration of the objective function.
    Configuration m_configuration;

    /// Spatial jacobian.
    Eigen::Matrix<double, 3, 3> m_space_jacobian;

    double m_initial_time;

    double m_joint_cost;

    double m_self_collision_cost;

    double m_workspace_cost;

    double m_energy_cost;

    double m_velocity_cost;

    double m_trajectory_cost;

    double m_manipulability_cost;

    double m_cost;
};

} // namespace FrankaRidgeback
