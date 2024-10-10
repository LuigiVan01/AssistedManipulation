#include "frankaridgeback/objective/track_point.hpp"

#include "frankaridgeback/dynamics.hpp"

#include <iostream>
#include <random>

namespace FrankaRidgeback {

double TrackPoint::get_cost(
    const VectorXd & s,
    const VectorXd & /*control */,
    mppi::Dynamics *d,
    double /*dt */
) {
    const State &state = s;
    auto dynamics = static_cast<Dynamics*>(d);

    double cost = point_cost(dynamics);

    if (m_configuration.enable_joint_limits) {
        cost += joint_limit_cost(state);
    }

    if (m_configuration.enable_self_collision_avoidance) {
        cost += self_collision_cost(dynamics);
    }

    if (m_configuration.enable_reach_limits) {
        cost += reach_cost(dynamics);
    }

    if (m_configuration.enable_power_limit) {
        cost += power_cost(dynamics);
    }

    return cost;
}

double TrackPoint::point_cost(Dynamics *dynamics)
{
    double distance = (
        dynamics->get_end_effector_state().position - m_configuration.point
    ).norm();

    return 100.0 * std::pow(distance, 2);
}

double TrackPoint::joint_limit_cost(const State &state)
{
    // Statically initialise the lower limits.
    static VectorXd lower_limit = []{
        VectorXd lower_limit = VectorXd(DoF::JOINTS);
        lower_limit <<
            -2.0, -2.0, -6.28,
            -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
            0.5, 0.5;
        return lower_limit;
    }();

    // Statically initialise the upper limits.
    static VectorXd upper_limit = []{
        VectorXd upper_limit(DoF::JOINTS);
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

double TrackPoint::self_collision_cost(Dynamics *dynamics)
{
    double cost = 0.0;

    // Self collision is implemented using imaginary spheres centered on each
    // joint. The joints are considered colliding if the spheres are
    // intersecting. The radii of the spheres can be adjusted to change the
    // sensitivity to self collision.
    const auto &self_collision = m_configuration.self_collision;

    // for (const auto &[first, second] : SELF_COLLISION_LINKS) {
    //     auto offset = dynamics->get_frame_offset(first, second);
    //     if (offset.norm() < self_collision.limit) {
    //         cost += self_collision.quadratic_cost * std::pow(
    //             self_collision.limit - offset.norm(), 2
    //         );
    //     }
    // }

    return cost;
}

double TrackPoint::power_cost(Dynamics *dynamics)
{
    const auto &power = m_configuration.maximum_power;
    return power.constant_cost * std::max(0.0, dynamics->get_power() - power.limit);
}

double TrackPoint::reach_cost(Dynamics *dynamics)
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

    return 0.0;
}

} // namespace FrankaRidgeback
