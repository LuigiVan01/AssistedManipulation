#include "logging/assisted_manipulation.hpp"

#include <numeric>
#include <ranges>
#include <limits>

namespace logger {

std::unique_ptr<AssistedManipulation> AssistedManipulation::create(
    Configuration configuration
) {
    using namespace std::string_literals;

    std::vector<std::string> logged;

    if (configuration.log_joint_limit)
        logged.push_back("joint_limit");

    if (configuration.log_minimise_velocity)
        logged.push_back("minimise_velocity");

    if (configuration.log_self_collision)
        logged.push_back("self_collision");

    if (configuration.log_trajectory)
        logged.push_back("trajectory");

    if (configuration.log_reach)
        logged.push_back("reach");

    if (configuration.log_power)
        logged.push_back("power");

    if (configuration.log_energy_tank)
        logged.push_back("energy_tank");

    if (configuration.log_manipulability)
        logged.push_back("manipulability");

    if (configuration.log_variable_damping)
        logged.push_back("variable_damping");

    auto logger = std::unique_ptr<AssistedManipulation>(
        new AssistedManipulation(configuration)
    );

    logger->m_logger = CSV::create(CSV::Configuration{
        .path = configuration.folder / "assisted_manipulation.csv",
        .header = CSV::make_header("time", logged, "total")
    });

    if (!logger->m_logger) {
        std::cerr << "failed to create csv logger" << std::endl;
        return nullptr;
    }

    logger->m_costs.resize(logged.size(), 0.0);

    return logger;
}

void AssistedManipulation::log(
    double time,
    const FrankaRidgeback::AssistedManipulation &objective
) {
    int i = 0;

    if (m_configuration.log_joint_limit)
        m_costs[i++] = objective.get_joint_limit_cost();

    if (m_configuration.log_minimise_velocity)
        m_costs[i++] = objective.get_joint_velocity_cost();

    if (m_configuration.log_self_collision)
        m_costs[i++] = objective.get_self_collision_cost();

    if (m_configuration.log_trajectory)
        m_costs[i++] = objective.get_trajectory_cost();

    if (m_configuration.log_reach)
        m_costs[i++] = objective.get_reach_cost();

    if (m_configuration.log_power)
        m_costs[i++] = objective.get_power_cost();

    if (m_configuration.log_energy_tank)
        m_costs[i++] = objective.get_energy_tank_cost();

    if (m_configuration.log_manipulability)
        m_costs[i++] = objective.get_manipulability_cost();

    if (m_configuration.log_variable_damping)
        m_costs[i++] = objective.get_variable_damping_cost();

    double total = std::accumulate(m_costs.begin(), m_costs.end(), 0.0);

    m_logger->write(time, m_costs, total);
}

} // namespace logger
