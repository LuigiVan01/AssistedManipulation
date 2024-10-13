#include "frankaridgeback/objective/assisted_manipulation.hpp"

#include <iostream>

#include "frankaridgeback/dynamics.hpp"

namespace FrankaRidgeback {

std::unique_ptr<AssistedManipulation> AssistedManipulation::create(
    const Configuration &configuration
) {
    return std::unique_ptr<AssistedManipulation>(
        new AssistedManipulation(configuration)
    );
}

AssistedManipulation::AssistedManipulation(
    const Configuration &configuration
  ) : m_configuration(configuration)
{
    reset(0.0);
}

void AssistedManipulation::reset(double time)
{
    m_initial_time = time;
    m_joint_cost = 0.0;
    m_minimise_velocity_cost = 0.0;
    m_self_collision_cost = 0.0;
    m_trajectory_cost = 0.0;
    m_workspace_cost = 0.0;
    m_joint_power_cost = 0.0;
    m_energy_tank_cost = 0.0;
    m_manipulability_cost = 0.0;
    m_variable_damping_cost = 0.0;
    m_cost = 0.0;
}

double AssistedManipulation::get_cost(
    const Eigen::VectorXd &s,
    const Eigen::VectorXd &c,
    mppi::Dynamics *d,
    double time
) {
    const State &state = s;
    const Control &control = c;

    auto dynamics = static_cast<Dynamics*>(d);

    double cost = 0.0;

    if (m_configuration.enable_joint_limit)
        cost += joint_limit_cost(state);

    if (m_configuration.enable_minimise_velocity)
        cost += minimise_velocity_cost(state);

    if (m_configuration.enable_self_collision)
        cost += self_collision_cost(dynamics);

    if (m_configuration.enable_trajectory_tracking)
        cost += trajectory_cost(dynamics, time);

    if (m_configuration.enable_workspace)
        cost += workspace_cost(dynamics);

    if (m_configuration.enable_minimise_joint_power)
        cost += power_cost(dynamics);

    if (m_configuration.enable_energy_tank)
        cost += energy_tank_cost(dynamics);

    if (m_configuration.enable_maximise_manipulability)
        cost += manipulability_cost(dynamics);

    if (m_configuration.enable_variable_damping)
        cost += variable_damping_cost(state);

    return cost;
}

double AssistedManipulation::joint_limit_cost(const State &state)
{
    double cost = 0.0;

    for (size_t i = 0; i < DoF::JOINTS; i++) {
        const auto &lower = m_configuration.lower_joint_limit[i];
        const auto &upper = m_configuration.upper_joint_limit[i];

        double position = state.position()(i);

        // Agressively penalise if joint limits breached.
        if (position < lower.limit) {
            cost += lower(std::fabs(lower.limit - position));
        }
        else if (position > upper.limit) {
            cost += upper(std::fabs(position - upper.limit));
        }
        else {
            // This tends to the middle of the joint limits, not really useful.
            // Rely on maximising manipulability instead.
            // cost += lower.linear_cost * std::pow(lower.limit - position, 2);
            // cost += upper.linear_cost * std::pow(upper.limit - position, 2);
        }
    }

    m_joint_cost += cost;
    return cost;
}

double AssistedManipulation::minimise_velocity_cost(const State &state)
{
    double cost = 0.0;

    for (size_t i = 0; i < DoF::JOINTS; i++) {
        const auto &objective = m_configuration.minimise_velocity[i];
        cost += objective.quadratic_cost * std::pow(std::fabs(state.velocity()(i)), 2);
    }

    m_minimise_velocity_cost += cost;
    return cost;
}

double AssistedManipulation::self_collision_cost(Dynamics *dynamics)
{
    static std::vector<std::tuple<Link, std::vector<Link>>> CHECK_COLLSION = {{
        {Link::PIVOT, {
            Link::PANDA_LINK3,
            Link::PANDA_LINK4,
            Link::PANDA_LINK5,
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK1, {
            Link::PANDA_LINK3,
            Link::PANDA_LINK4,
            Link::PANDA_LINK5,
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK2, {
            Link::PANDA_LINK4,
            Link::PANDA_LINK5,
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK3, {
            Link::PANDA_LINK5,
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK4, {
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK5, {
            Link::PANDA_LINK7,
        }}
    }};

    const auto &self_collision = m_configuration.self_collision_limit;
    double cost = 0.0;

    for (const auto &[first, against] : CHECK_COLLSION) {
        auto &first_constraint = m_configuration.self_collision_limit[(std::size_t)first - 3];

        for (auto second : against) {
            auto &second_constraint = m_configuration.self_collision_limit[(std::size_t)second - 3];

            // Distance between sphere origins.
            double distance = (
                dynamics->get_link_position(second) -
                dynamics->get_link_position(first)
            ).norm();

            // Sum of sphere radii.
            double radii = first_constraint.limit + second_constraint.limit;

            // Collision vector. Positive when colliding.
            double collision = radii - distance;

            if (collision > 0) {
                cost += first_constraint.quadratic_cost * std::pow(collision, 2);
                cost += second_constraint.quadratic_cost * std::pow(collision, 2);
            }
        }
    }

    m_self_collision_cost += cost;
    return cost;
}

double AssistedManipulation::trajectory_cost(const Dynamics *dynamics, double time)
{
    if (!dynamics->get_forecast())
        return 0.0;

    const auto &state = dynamics->get_end_effector_state();
    const auto forecast = dynamics->get_forecast()->get();

    Vector3d force = forecast->get_end_effector_wrench(time).head<3>();
    Vector3d initial_position = forecast->get_end_effector_trajectory()[0].position;
    Vector3d target = initial_position + (force / 100).cwiseMin(0.5).cwiseMax(-0.5);
    Vector3d target_vector = target - state.position;
    double distance = target_vector.norm();

    // const auto &forecast_state = forecast->get_end_effector_state(time);
    // double distance = (state.position - forecast_state.position).norm();

    double cost = 0.0;
    if (distance > m_configuration.trajectory_position.limit) {
        cost += m_configuration.trajectory_position(distance);

        // Project linear velocity onto the force vector.
        double projection = (
            state.linear_velocity.dot(target_vector) /
            target_vector.dot(target_vector)
        );

        double projected = std::copysign(1.0, projection) * (target_vector * projection).norm();

        // Make relative to the trajectory limit.
        projection = std::max(m_configuration.trajectory_velocity.limit - projected, 0.0);
        projection = std::exp(projection);
        cost += m_configuration.trajectory_velocity(projection) / 2;
    }

    m_trajectory_cost += cost;
    return cost;
}

double AssistedManipulation::power_cost(Dynamics *dynamics)
{
    double cost = 0.0;
    double power = dynamics->get_joint_power();

    if (power > 0) {
        double cost = m_configuration.minimise_joint_power(std::fabs(power));
        m_joint_power_cost += cost;
    }

    return cost;
}

double AssistedManipulation::energy_tank_cost(Dynamics *dynamics)
{
    const auto &maximum = m_configuration.maximum_energy;
    double energy = dynamics->get_tank_energy();

    double cost = 0.0;
    if (energy < 0.1) {
        cost += (
            maximum.constant_cost +
            maximum.quadratic_cost * std::pow(energy - maximum.limit, 2)
        );
    }
    else if (energy > maximum.limit) {
        cost += cost += (
            maximum.constant_cost +
            maximum.quadratic_cost * std::pow(energy - maximum.limit, 2)
        );
    }

    m_energy_tank_cost += cost;
    return cost;
}

double AssistedManipulation::manipulability_cost(Dynamics *dynamics)
{
    const auto &minimum = m_configuration.minimum_manipulability;
    const auto &jacobian = dynamics->get_end_effector_state().jacobian.rightCols(
        DoF::JOINTS - DoF::BASE
    ).topLeftCorner(3, DoF::ARM);

    m_space_jacobian = jacobian * jacobian.transpose();

    // Value proportional to the volume of the manipulability ellipsoid.
    double ellipsoid_volume = std::sqrt(m_space_jacobian.determinant());

    if (std::isnan(ellipsoid_volume))
        ellipsoid_volume = 1e-5;
    else if (ellipsoid_volume < 1e-5)
        ellipsoid_volume = 1e-5;
    else if (ellipsoid_volume > 1e5)
        ellipsoid_volume = 1e5;

    double cost = minimum.quadratic_cost * std::pow(1 / ellipsoid_volume, 2);

    m_manipulability_cost += cost;
    return cost;
}

double AssistedManipulation::workspace_cost(Dynamics *dynamics)
{
    const auto &objective = m_configuration.workspace;
    double cost = 0.0;

    // Position of the end effector.
    Vector3d end_effector = dynamics->get_end_effector_state().position;

    // Rotation from world frame to infront of the robot.
    AngleAxisd rotate_forward = AngleAxisd(dynamics->get_state()[2], Vector3d::UnitZ());

    // Unit vector in the forward direction of the robot, normal to the
    // infront / behind plane.
    Vector3d forward = rotate_forward * Vector3d::UnitX();

    // Position of the plane in front of the robot.
    Vector3d robot = (
        dynamics->get_frame_position(Frame::ARM_MOUNT_JOINT) +
        rotate_forward * Vector3d(0.3, 0, 0.15) // offset from base link
    );

    // Vector from plane origin to end effector.
    Vector3d to_end_effector = end_effector - robot;

    // Distance from the infront / behind plane to the end effector.
    double projection = (
        to_end_effector.dot(forward) / forward.dot(forward)
    );

    // Keep end effector in front of the robot.
    if (projection < 0) {
        cost += objective(projection);
    }

    // Limit reach.
    double reach = to_end_effector.norm();
    if (reach > m_configuration.workspace_maximum_reach) {
        cost += objective(
            reach - m_configuration.workspace_maximum_reach
        );
    }

    // Calculate yaw between body and end effector.
    Vector2d v1 = to_end_effector.head<2>();
    Vector2d v2 = forward.head<2>();
    double yaw = std::acos(v1.dot(v2) / v1.norm() / v2.norm());
    if (!std::isnan(yaw)) {
        cost += m_configuration.workspace_yaw(std::fabs(yaw));
    }

    // Keep the end effector above the robot.
    double height = end_effector[2] - robot[2];
    if (height < 0) {
        cost += objective(height);
    }

    m_workspace_cost += cost;
    return cost;
}

double AssistedManipulation::variable_damping_cost(const State &state)
{
    // double velocity = m_model->end_effector_velocity().norm();

    // double expected_damping = (
    //     m_configuration.variable_damping_maximum *
    //     std::exp(-m_configuration.variable_damping_dropoff * velocity)
    // );

    // // The expected resistive force against the external force.
    // double expected_inertia = expected_damping * velocity;

    // double inertia = m_model->get_data().Jm_model->get_data()->M
    // // Subtract the 
    // double residual_force = (
    //     state.end_effector_force().norm() - expected_damping * velocity
    // );

    // // // F = c * x_dot therefore c = F \ x_dot
    // double damping = state.end_effector_torque().norm() / velocity;

    // m_variable_damping_cost.quadratic * ;

    return 0.0;
}

} // namespace FrankaRidgeback
