#include "frankaridgeback/objective/track_point.hpp"

#include "frankaridgeback/dynamics.hpp"

#include <iostream>
#include <random>

const TrackPoint::Configuration TrackPoint::DEFAULT_CONFIGURATION {
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

double TrackPoint::get_cost(
    const VectorXd & s,
    const VectorXd & /*control */,
    mppi::Dynamics *d,
    double /*dt */
) {
    const FrankaRidgeback::State &state = s;
    auto dynamics = static_cast<FrankaRidgeback::Dynamics*>(d);

    double cost = point_cost(dynamics);

    if (m_configuration.enable_joint_limits) {
        cost += joint_cost(state);
    }

    if (m_configuration.enable_power_limit) {
        cost += power_cost(dynamics);
    }

    return cost;
}

double TrackPoint::point_cost(FrankaRidgeback::Dynamics *dynamics)
{
    return 100.0 * std::pow((dynamics->get_end_effector_position() - m_configuration.point).norm(), 2);
}

double TrackPoint::joint_cost(const FrankaRidgeback::State &state)
{
    // Statically initialise the lower limits.
    static VectorXd lower_limit = []{
        VectorXd lower_limit = VectorXd(FrankaRidgeback::DoF::JOINTS);
        lower_limit <<
            -2.0, -2.0, -6.28,
            -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
            0.5, 0.5;
        return lower_limit;
    }();

    // Statically initialise the upper limits.
    static VectorXd upper_limit = []{
        VectorXd upper_limit(FrankaRidgeback::DoF::JOINTS);
        upper_limit <<
            2.0, 2.0, 6.28,
            2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973,
            0.5, 0.5;
        return upper_limit;
    }();

    double cost = 0.0;

    // Joint limits.
    for (size_t i = 0; i < 10; i++) {
        if (state(i) < lower_limit[i])
            cost += 1000.0 + 100000.0 * std::pow(lower_limit[i] - state(i), 2);

        if (state(i) > upper_limit[i])
            cost += 1000.0 + 100000.0 * std::pow(state(i) - upper_limit[i], 2);
    }

    return cost;
}

double TrackPoint::power_cost(FrankaRidgeback::Dynamics *dynamics)
{
    const auto &power = m_configuration.maximum_power;
    return power.constant_cost * std::max(0.0, dynamics->get_power() - power.limit);
}
