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
    reset();
}

void AssistedManipulation::reset()
{
    m_joint_cost = 0.0;
    m_minimise_velocity_cost = 0.0;
    m_self_collision_cost = 0.0;
    m_trajectory_cost = 0.0;
    m_reach_cost = 0.0;
    m_power_cost = 0.0;
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

    if (m_configuration.enable_reach_limit)
        cost += reach_cost(dynamics);

    if (m_configuration.enable_maximum_power)
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

        if (position < lower.limit) {
            cost += (
                lower.constant_cost +
                lower.quadratic_cost * std::pow(lower.limit - position, 2)
            );
        }
        else if (position > upper.limit) {
            cost += (
                upper.constant_cost +
                upper.quadratic_cost * std::pow(position - upper.limit, 2)
            );
        }
    }

    m_joint_cost += cost;
    return cost;
}

double AssistedManipulation::minimise_velocity_cost(const State &state)
{
    const auto &objective = m_configuration.minimise_velocity;
    double cost = 0.0;

    for (size_t i = 0; i < DoF::JOINTS; i++) {
        cost += objective.quadratic_cost * std::fabs(state.velocity()(i));
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

double AssistedManipulation::trajectory_cost(
    Dynamics *dynamics,
    double time
) {
    if (!dynamics->get_forecast().has_value())
        return 0.0;

    const auto &state = dynamics->get_end_effector_state();
    const auto &forecast_state = dynamics->get_forecast().value()->get()->get_end_effector_state(time);

    double position_difference = (state.position - forecast_state.position).norm();
    double velocity_difference = (state.linear_velocity - forecast_state.linear_velocity).norm();

    const auto &objective = m_configuration.trajectory;
    double cost = (
        objective.quadratic_cost * std::pow(position_difference, 2) +
        objective.quadratic_cost * std::pow(velocity_difference, 2)
    );

    m_trajectory_cost += cost;
    return cost;
}

double AssistedManipulation::power_cost(Dynamics *dynamics)
{
    const auto &maximum = m_configuration.maximum_power;

    double power = dynamics->get_power();
    if (power < maximum.limit)
        return 0.0;

    double cost = (
        maximum.constant_cost +
        std::max(0.0, maximum.quadratic_cost * (power - maximum.limit))
    );

    m_power_cost += cost;
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
    const auto &jacobian = dynamics->get_end_effector_state().jacobian;

    m_space_jacobian = jacobian * jacobian.transpose();

    // Value proportional to the volumne of the manipulability ellipsoid.
    double ellipsoid_volume = std::sqrt(m_space_jacobian.determinant());

    // Clip to a small value to prevent division by zero on singularity.
    if (ellipsoid_volume < 1e-10)
        ellipsoid_volume = 1e-10;

    double cost = minimum.quadratic_cost * std::pow(1 / ellipsoid_volume, 2);

    m_manipulability_cost += cost;
    return cost;
}

double AssistedManipulation::reach_cost(Dynamics *dynamics)
{
    double cost = 0.0;
    const auto &min = m_configuration.minimum_reach;
    const auto &max = m_configuration.maximum_reach;

    double offset = 0.0;
    // dynamics->get_frame_offset(
    //     Frame::BASE_LINK_JOINT,
    //     Frame::PANDA_GRASP_JOINT
    // ).norm();

    if (offset < min.limit) {
        cost += (
            min.constant_cost +
            min.quadratic_cost * std::pow(offset - min.limit, 2)
        );
    }
    else if (offset > max.limit) {
        cost += (
            max.constant_cost +
            max.quadratic_cost * std::pow(offset - max.limit, 2)
        );
    }

    m_reach_cost += cost;
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
