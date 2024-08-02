#include "logging/mppi.hpp"

#include <numeric>
#include <ranges>

namespace logger {

std::unique_ptr<MPPI> MPPI::create(Configuration &&configuration)
{
    using namespace std::string_literals;

    std::vector<std::string> states, control, rollouts;

    // for (int i = 0; i < configuration.trajectory->get_state_dof(); i++)
    //     states.push_back("state"s + std::to_string(i));

    for (int i = 1; i < configuration.trajectory->get_control_dof() + 1; i++)
        control.push_back("control"s + std::to_string(i));

    for (int i = 1; i < configuration.trajectory->get_rollout_count() + 1; i++)
        rollouts.push_back("rollout"s + std::to_string(i));

    std::vector<std::string> cost_header {"update"};
    cost_header.insert(cost_header.end(), rollouts.begin(), rollouts.end());

    auto costs = CSV::create(CSV::Configuration{
        .path = configuration.folder / "costs.csv",
        .header = std::move(cost_header)
    });

    std::vector<std::string> weights_header {"update"};
    weights_header.insert(weights_header.end(), rollouts.begin(), rollouts.end());

    auto weights = CSV::create(CSV::Configuration{
        .path = configuration.folder / "weights.csv",
        .header = std::move(weights_header)
    });

    std::vector<std::string> gradient_header {"update", "time"};
    gradient_header.insert(gradient_header.end(), control.begin(), control.end());

    auto gradient = CSV::create(CSV::Configuration{
        .path = configuration.folder / "gradient.csv",
        .header = std::move(gradient_header)
    });

    std::vector<std::string> optimal_rollout_header {"update", "time"};
    optimal_rollout_header.insert(optimal_rollout_header.end(), control.begin(), control.end());

    auto optimal_rollout = CSV::create(CSV::Configuration{
        .path = configuration.folder / "optimal_rollout.csv",
        .header = std::move(optimal_rollout_header)
    });

    auto optimal_cost = CSV::create(CSV::Configuration{
        .path = configuration.folder / "optimal_cost.csv",
        .header = {"update", "cost"}
    });

    auto update = CSV::create(CSV::Configuration{
        .path = configuration.folder / "update.csv",
        .header = {"update", "time", "update_duration"}
    });

    if (!costs || !weights || !gradient || !optimal_rollout || !optimal_cost || !update) {
        std::cerr << "failed to create csv logger" << std::endl;
        return nullptr;
    }

    return std::unique_ptr<MPPI>(
        new MPPI(
            std::move(costs),
            std::move(weights),
            std::move(gradient),
            std::move(optimal_rollout),
            std::move(optimal_cost),
            std::move(update)
        )
    );
}

MPPI::MPPI(
    std::unique_ptr<CSV> &&costs,
    std::unique_ptr<CSV> &&weights,
    std::unique_ptr<CSV> &&gradient,
    std::unique_ptr<CSV> &&optimal_rollout,
    std::unique_ptr<CSV> &&optimal_cost,
    std::unique_ptr<CSV> &&update
) : m_last_update(-9999.0)
  , m_costs(std::move(costs))
  , m_weights(std::move(weights))
  , m_gradient(std::move(gradient))
  , m_optimal_rollout(std::move(optimal_rollout))
  , m_optimal_cost(std::move(optimal_cost))
  , m_update(std::move(update))
{}

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

    m_last_update = time;
}

} // namespace logger
