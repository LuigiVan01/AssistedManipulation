#include "frankaridgeback/dynamics.hpp"

#include <limits>

namespace FrankaRidgeback {

std::unique_ptr<DynamicsForecast> DynamicsForecast::create(
    const Configuration &configuration,
    std::unique_ptr<Dynamics> &&dynamics
) {
    auto end_effector_wrench_forecast = Forecast::create(
        configuration.end_effector_wrench_forecast
    );
    if (!end_effector_wrench_forecast) {
        std::cerr << "failed to create forecast for end effector wrench" << std::endl;
        return nullptr;
    }

    unsigned int steps = std::ceil(configuration.horison / configuration.time_step);
    if (steps <= 0) {
        std::cerr << "time horison is too small for time step" << std::endl;
        return nullptr;
    }

    return std::unique_ptr<DynamicsForecast>(
        new DynamicsForecast(
            configuration,
            std::move(dynamics),
            std::move(end_effector_wrench_forecast),
            steps
        )
    );
}

DynamicsForecast::DynamicsForecast(
    const Configuration &configuration,
    std::unique_ptr<Dynamics> &&dynamics,
    std::unique_ptr<Forecast> &&end_effector_wrench_forecast,
    unsigned int steps
    ) : m_configuration(configuration)
      , m_steps(steps)
      , m_last_forecast(std::numeric_limits<double>::min())
      , m_dynamics(std::move(dynamics))
      , m_end_effector_wrench_forecast(std::move(end_effector_wrench_forecast))
      , m_end_effector(steps, EndEffectorState())
      , m_end_effector_wrench(steps, Vector6d::Zero())
{}

const EndEffectorState &DynamicsForecast::get_end_effector_state(double time)
{
    // Extrapolate initial wrench backwards.
    if (time < m_last_forecast)
        return m_end_effector.front();

    // Extrapolate last wrench forwards.
    if (time > m_configuration.horison)
        return m_end_effector.back();

    // Steps into the horison.
    double t = (time - m_last_forecast) / m_configuration.time_step;

    // The step less than or equal to t.
    int lower = (int)t;

    return m_end_effector[lower];
}

Vector6d DynamicsForecast::get_end_effector_wrench(double time) const
{
    return m_end_effector_wrench_forecast->forecast(time);
}

void DynamicsForecast::forecast(State state, double time)
{
    auto control = Control::Zero();
    m_dynamics->set_state(state, time);

    for (unsigned int step = 0; step < m_steps; ++step) {
        double t = time + step * m_configuration.time_step;

        Vector6d wrench = m_end_effector_wrench_forecast->forecast(t);
        m_end_effector_wrench[step] = wrench;

        // Simulate the forecast wrench trajectory.
        m_dynamics->add_end_effector_simulated_wrench(wrench);

        // Step the dynamics simulation.
        m_dynamics->step(control, m_configuration.time_step);
        m_end_effector[step] = m_dynamics->get_end_effector_state();
    }
}

} // namespace FrankaRidgeback
