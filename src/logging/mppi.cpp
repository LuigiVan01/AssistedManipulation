#include "logging/mppi.hpp"

#include <numeric>
#include <ranges>
#include <limits>

namespace logger {

std::unique_ptr<MPPI> MPPI::create(const Configuration &configuration)
{
    using namespace std::string_literals;

    std::vector<std::string> states, control, rollouts;

    for (int i = 1; i < configuration.control_dof + 1; i++)
        control.push_back("control"s + std::to_string(i));

    for (int i = 1; i < configuration.rollouts + 1; i++)
        rollouts.push_back("rollout"s + std::to_string(i));

    auto mppi = std::unique_ptr<MPPI>(new MPPI());

    if (configuration.log_costs) {
        mppi->m_costs = CSV::create(CSV::Configuration{
            .path = configuration.folder / "costs.csv",
            .header = CSV::make_header("update", rollouts)
        });
    }

    if (configuration.log_weights) {
        mppi->m_weights = CSV::create(CSV::Configuration{
            .path = configuration.folder / "weights.csv",
            .header = CSV::make_header("update", rollouts)
        });
    }

    if (configuration.log_gradient) {
        mppi->m_gradient = CSV::create(CSV::Configuration{
            .path = configuration.folder / "gradient.csv",
            .header = CSV::make_header("update", "time", control)
        });
    }

    if (configuration.log_optimal_rollout) {
        mppi->m_optimal_rollout = CSV::create(CSV::Configuration{
            .path = configuration.folder / "optimal_rollout.csv",
            .header = CSV::make_header("update", "time", control)
        });
    }

    if (configuration.log_optimal_cost) {
        mppi->m_optimal_cost = CSV::create(CSV::Configuration{
            .path = configuration.folder / "optimal_cost.csv",
            .header = CSV::make_header("update", "cost")
        });
    }

    if (configuration.log_update) {
        mppi->m_update = CSV::create(CSV::Configuration{
            .path = configuration.folder / "update.csv",
            .header = CSV::make_header("update", "time", "update_duration")
        });
    }

    bool error = (
        (configuration.log_costs && !mppi->m_costs) ||
        (configuration.log_weights && !mppi->m_weights) ||
        (configuration.log_gradient && !mppi->m_gradient) ||
        (configuration.log_optimal_rollout && !mppi->m_optimal_rollout) ||
        (configuration.log_optimal_cost && !mppi->m_optimal_cost) ||
        (configuration.log_update && !mppi->m_update)
    );

    if (error) {
        std::cerr << "failed to create csv logger" << std::endl;
        return nullptr;
    }

    mppi->m_last_update = -std::numeric_limits<double>::min();

    return mppi;
}

void MPPI::log(const mppi::Trajectory &trajectory)
{
    auto time = trajectory.get_update_last();

    if (time == m_last_update)
        return;

    auto step = trajectory.get_time_step();
    auto steps = trajectory.get_step_count();
    std::size_t iteration = trajectory.get_update_count();

    if (m_update) {
        m_update->write(
            iteration,
            time,
            trajectory.get_update_duration()
        );
    }

    // No op if already the correct size.
    m_time.resize(steps);

    // Update the time horison.
    for (int i = 0; i < steps; ++i)
        m_time[i] = time + i * step;

    if (m_costs) {
        auto costs = std::ranges::transform_view(
            trajectory.get_rollouts(),
            [](const auto &rollout) { return rollout.cost; }
        );
        m_costs->write(iteration, costs);
    }

    if (m_weights) {
        m_weights->write(iteration, trajectory.get_weights());
    }

    if (m_gradient) {
        for (int i = 0; i < steps; ++i)
            m_gradient->write(iteration, m_time[i], trajectory.get_gradient().col(i));
    }

    if (m_optimal_rollout) {
        for (int i = 0; i < steps; ++i)
            m_optimal_rollout->write(iteration, m_time[i], trajectory.get_optimal_rollout().col(i));
    }

    if (m_optimal_cost) {
        m_optimal_cost->write(iteration, trajectory.get_optimal_cost());
    }

    m_last_update = time;
}

} // namespace logger
