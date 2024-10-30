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

    if (configuration.log_self_collision_limit)
        logged.push_back("self_collision");

    if (configuration.log_workspace_limit)
        logged.push_back("workspace");

    if (configuration.log_energy_limit)
        logged.push_back("energy_tank");

    if (configuration.log_velocity_cost)
        logged.push_back("minimise_velocity");

    if (configuration.log_trajectory_cost)
        logged.push_back("trajectory");

    if (configuration.log_manipulability_cost)
        logged.push_back("manipulability");

    if (configuration.log_total)
        logged.push_back("total");

    auto logger = std::unique_ptr<AssistedManipulation>(
        new AssistedManipulation(configuration)
    );

    logger->m_logger = CSV::create(CSV::Configuration{
        .path = configuration.folder / "assisted_manipulation.csv",
        .header = CSV::make_header("time", logged)
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
    if (time == m_last_update)
        return;

    int i = 0;

    if (m_configuration.log_joint_limit)
        m_costs[i++] = objective.get_joint_limit_cost();

    if (m_configuration.log_self_collision_limit)
        m_costs[i++] = objective.get_self_collision_cost();

    if (m_configuration.log_workspace_limit)
        m_costs[i++] = objective.get_workspace_cost();

    if (m_configuration.log_energy_limit)
        m_costs[i++] = objective.get_energy_tank_cost();

    if (m_configuration.log_velocity_cost)
        m_costs[i++] = objective.get_joint_velocity_cost();

    if (m_configuration.log_trajectory_cost) {
        double c = objective.get_trajectory_cost();
        // std::cout << c << std::endl;
        m_costs[i++] = c;
    }

    if (m_configuration.log_manipulability_cost)
        m_costs[i++] = objective.get_manipulability_cost();

    if (m_configuration.log_total) {
        m_costs[i] = 0.0;
        double cost = std::accumulate(m_costs.begin(), m_costs.end(), 0.0);
        std::cout << cost << std::endl;
        m_costs[i++] = cost;
    }

    m_logger->write(time, m_costs);
    m_logger->flush();

    m_last_update = time;
}

} // namespace logger
