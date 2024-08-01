#include "logging/log_mppi.hpp"

void MPPILogger::log(const mppi::Trajectory &trajectory)
{
    auto time = trajectory.get_update_last();
    auto step = trajectory.get_time_step();
    auto steps = trajectory.get_steps();

    std::size_t iteration = trajectory.get_update_count();

    if (m_configuration.iteration) {
        m_configuration.iteration->log(
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

    if (m_configuration.costs) {
        for (int i = 0; i < trajectory.get_rollouts().size(); i++)
            m_configuration.costs->log(iteration, i, trajectory.get_rollouts()[i].cost);
    }

    if (m_configuration.weights) {
        for (int i = 0; i < steps; ++i)
            m_configuration.weights->log(iteration, m_time[i], trajectory.get_weights().col(i));
    }

    if (m_configuration.gradient) {
        for (int i = 0; i < steps; ++i)
            m_configuration.gradient->log(iteration, m_time[i], trajectory.get_gradient().col(i));
    }

    // Log the optimal rollout.
    if (m_configuration.optimal_rollout) {
        for (int i = 0; i < steps; ++i)
            m_configuration.optimal_rollout->log(iteration, m_time[i], trajectory.get_optimal_rollout().col(i));
    }
}
