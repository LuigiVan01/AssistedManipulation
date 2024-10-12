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

        /// If the joint velocities should be minimised.
        bool enable_minimise_velocity;

        /// If self collision costs are enabled.
        bool enable_self_collision;

        /// If workspace costs are enabled.
        bool enable_workspace;

        /// If tracking the expected trajectory is enabled.
        bool enable_trajectory_tracking;

        /// If end effector manipulability is maximised.
        bool enable_maximise_manipulability;

        /// If the power used by the trajectory is minimised.
        bool enable_maximum_power;

        /// If variable damping should be enabled.
        bool enable_variable_damping;

        /// If energy tank should be enabled.
        bool enable_energy_tank;

        /// Lower joint limits if enabled.
        std::array<QuadraticCost, DoF::JOINTS> lower_joint_limit;

        /// Upper joint limits if enabled.
        std::array<QuadraticCost, DoF::JOINTS> upper_joint_limit;

        /// Velocity minimisation cost.
        std::array<QuadraticCost, DoF::JOINTS> minimise_velocity;

        /// Self collision cost.
        std::array<QuadraticCost, 8> self_collision_limit;

        /// Trajectory not infront cost.
        QuadraticCost workspace;

        /// Cost of matching the forecast trajectory.
        QuadraticCost trajectory;

        /// Manipulability limits if enabled.
        QuadraticCost minimum_manipulability;

        /// Maximum power (joules per second) usage if enabled. 
        QuadraticCost maximum_power;

        /// Maximum energy tank energy if enabled.
        QuadraticCost maximum_energy;

        /// The maximum damping that occurs when the end effector has zero
        /// velocity. The A in c(v) = Ae^{lambda * v}
        double variable_damping_maximum;

        /// The exponential drop-off from variable_damping_maximum with respect
        /// to velocity. The lambda in c(v) = Ae^{lambda * v}
        double variable_damping_dropoff;

        /// Constant multiplier of time.
        double trajectory_time_factor;

        // JSON conversion for assisted manipulation objective configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            enable_minimise_velocity,
            enable_joint_limit,
            enable_self_collision,
            enable_workspace,
            enable_trajectory_tracking,
            enable_maximise_manipulability,
            enable_maximum_power,
            enable_variable_damping,
            enable_energy_tank,
            lower_joint_limit,
            upper_joint_limit,
            minimise_velocity,
            self_collision_limit,
            workspace,
            trajectory,
            minimum_manipulability,
            maximum_power,
            maximum_energy,
            variable_damping_maximum,
            variable_damping_dropoff,
            trajectory_time_factor
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
        .enable_minimise_velocity = true,
        .enable_self_collision = true,
        .enable_workspace = true,
        .enable_trajectory_tracking = true,
        .enable_maximise_manipulability = false,
        .enable_maximum_power = false,
        .enable_variable_damping = false,
        .enable_energy_tank = false,
        .lower_joint_limit = {{
            {-2.0,    1'000, 0.0,  100'000}, // Base x
            {-2.0,    1'000, 0.0,  100'000}, // Base y
            {-6.28,   1'000, 0.0,  100'000}, // Base yaw
            {-2.8,    1'000, 10.0, 100'000}, // Joint1
            {-1.745,  1'000, 50.0, 100'000}, // Joint2
            {-2.8,    1'000, 10.0, 100'000}, // Joint3
            {-3.0718, 1'000, 10.0, 100'000}, // Joint4
            {-2.7925, 1'000, 10.0, 100'000}, // Joint5
            {0.349,   1'000, 10.0, 100'000}, // Joint6
            {-2.967,  1'000, 10.0, 100'000}, // Joint7
            {0.0,     1'000, 10.0, 100'000}, // Gripper x
            {0.0,     1'000, 10.0, 100'000}  // Gripper y
        }},
        .upper_joint_limit = {{
            {2.0,     1'000, 0.0, 100'000.0}, // Base x
            {2.0,     1'000, 0.0, 100'000.0}, // Base y
            {6.28,    1'000, 0.0, 100'000.0}, // Base yaw
            {2.8,     1'000, 10.0, 100'000.0}, // Joint1
            {1.745,   1'000, 50.0, 100'000.0}, // Joint2
            {2.8,     1'000, 10.0, 100'000.0}, // Joint3
            {0.0,     1'000, 10.0, 100'000.0}, // Joint4
            {2.7925,  1'000, 10.0, 100'000.0}, // Joint5
            {4.53785, 1'000, 10.0, 100'000.0}, // Joint6
            {2.967,   1'000, 10.0, 100'000.0}, // Joint7
            {0.5,     1'000, 10.0, 100'000.0}, // Gripper x
            {0.5,     1'000, 10.0, 100'000.0}  // Gripper y
        }},
        .minimise_velocity = {{
            {0.0, 0.0, 0.0, 10000.0},   // Base x
            {0.0, 0.0, 0.0, 10000.0},   // Base y
            {0.0, 0.0, 0.0, 100.0},   // Base yaw
            {0.0, 0.0, 0.0, 10.0},    // Joint1
            {0.0, 0.0, 0.0, 10.0},    // Joint2
            {0.0, 0.0, 0.0, 10.0},    // Joint3
            {0.0, 0.0, 0.0, 10.0},    // Joint4
            {0.0, 0.0, 0.0, 10.0},    // Joint5
            {0.0, 0.0, 0.0, 10.0},    // Joint6
            {0.0, 0.0, 0.0, 10.0},    // Joint7
            {0.0, 0.0, 0.0, 1000.0}, // Gripper x
            {0.0, 0.0, 0.0, 1000.0}  // Gripper y
        }},
        .self_collision_limit = {{
            {0.75, 1000.0, 0.0, 100'000}, // Base link
            {0.1,  1000.0, 0.0, 100'000}, // Arm link 1
            {0.1,  1000.0, 0.0, 100'000}, // Arm link 2
            {0.1,  1000.0, 0.0, 100'000}, // Arm link 3
            {0.1,  1000.0, 0.0, 100'000}, // Arm link 4
            {0.1,  1000.0, 0.0, 100'000}, // Arm link 5
            {0.1,  1000.0, 0.0, 100'000}, // Arm link 6
            {0.1,  1000.0, 0.0, 100'000}  // Arm link 7
        }},
        .workspace = {
            .limit = 0.0,
            .constant_cost = 100,
            .linear_cost = 500,
            .quadratic_cost = 100'000
        },
        .trajectory = {
            .limit = 0.01, // Within 1cm of target.
            .constant_cost = 100, // Prevents constant error around target.
            .quadratic_cost = 200'000.0 // Double limiting constraints.
        },
        .minimum_manipulability = {
            .limit = 1.0,
            .quadratic_cost = 100
        },
        .maximum_power = {
            .limit = 100.0,
            .constant_cost = 100,
            .quadratic_cost = 10'000
        },
        .maximum_energy = {},
        .variable_damping_maximum = 0.0,
        .variable_damping_dropoff = 0.0,
        .trajectory_time_factor = 0
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

    inline double get_joint_velocity_cost() const {
        return m_minimise_velocity_cost;
    }

    inline double get_self_collision_cost() const {
        return m_self_collision_cost;
    }

    inline double get_trajectory_cost() const {
        return m_trajectory_cost;
    }

    inline double get_workspace_cost() const {
        return m_workspace_cost;
    }

    inline double get_power_cost() const {
        return m_power_cost;
    }

    inline double get_energy_tank_cost() const {
        return m_energy_tank_cost;
    }

    inline double get_manipulability_cost() const {
        return m_manipulability_cost;
    }

    inline double get_variable_damping_cost() const {
        return m_variable_damping_cost;
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

    double minimise_velocity_cost(const State &state);

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
    double trajectory_cost(const Dynamics *dynamics, double time);

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
    double workspace_cost(Dynamics *dynamics);

    /**
     * @brief Penalises dynamics states that exceed the power limit.
     * 
     * @param dynamics The current dynamics state.
     * @returns A cost penalising dynamics that go over the maximum power draw.
     */
    double power_cost(Dynamics *dynamics);

    /**
     * @brief Penalises trajectories that deplete the energy tank and maximum
     * tank energy.
     * 
     * @param dynamics Pointer to the dynamics object.
     * @returns The cost. 
     */
    double energy_tank_cost(Dynamics *dynamics);

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
    Eigen::Matrix<double, 3, 3> m_space_jacobian;

    double m_initial_time;

    double m_joint_cost;

    double m_minimise_velocity_cost;

    double m_self_collision_cost;

    double m_trajectory_cost;

    double m_workspace_cost;

    double m_power_cost;

    double m_energy_tank_cost;

    double m_manipulability_cost;

    double m_variable_damping_cost;

    double m_cost;
};

} // namespace FrankaRidgeback
