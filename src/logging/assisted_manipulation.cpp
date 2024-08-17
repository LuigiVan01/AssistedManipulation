#include "logging/assisted_manipulation.hpp"

#include <numeric>
#include <ranges>
#include <limits>

namespace logger {

std::unique_ptr<AssistedManipulation> AssistedManipulation::create(Configuration &&configuration)
{
    using namespace std::string_literals;

    std::vector<std::string> logged;

    if (configuration.log_joint_cost)
        logged.push_back("joint_cost");

    if (configuration.log_reach_cost)
        logged.push_back("reach_cost");

    if (configuration.log_manipulability_cost)
        logged.push_back("manipulability_cost");

    if (configuration.log_power_cost)
        logged.push_back("power_cost");

    if (configuration.log_variable_damping_cost)
        logged.push_back("variable_damping_cost");

    if (logged.empty()) {
        std::cerr << "no assisted manipulation costs logged" << std::endl;
        return nullptr;
    }

    auto logger = std::unique_ptr<AssistedManipulation>(new AssistedManipulation());

    logger->m_logger = CSV::create(CSV::Configuration{
        .path = configuration.folder / "costs.csv",
        .header = CSV::make_header("update", logged)
    });

    if (!logger->m_logger) {
        std::cerr << "failed to create csv logger" << std::endl;
        return nullptr;
    }

    logger->m_costs.resize(logged.size(), 0.0);

    return logger;
}

void AssistedManipulation::log(
    const mppi::Trajectory &trajectory,
    const ::AssistedManipulation &objective
) {
    int i = 0;

    if (configuration.log_joint_cost)
        m_costs[i++] = objective.get_joint_cost();

    if (configuration.log_reach_cost)
        m_costs[i++] = objective.get_reach_cost();

    if (configuration.log_manipulability_cost)
        m_costs[i++] = objective.get_manipulability_cost();

    if (configuration.log_power_cost)
        m_costs[i++] = objective.get_power_cost();

    if (configuration.log_variable_damping_cost)
        m_costs[i++] = objective.variable_damping_cost();

    m_logger->write(trajectory.get_update_count(), m_costs);
}

} // namespace logger
