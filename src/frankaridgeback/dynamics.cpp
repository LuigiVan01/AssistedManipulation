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
      , m_joint_position(steps, Eigen::Vector<double, DoF::JOINTS>::Zero())
      , m_end_effector(steps, EndEffectorState())
      , m_power(steps, 0.0)
      , m_energy(steps, 0.0)
      , m_end_effector_wrench(steps, Vector6d::Zero())
{}

void DynamicsForecast::forecast(State state, double time)
{
    auto control = Control::Zero();
    m_dynamics->set_state(state, time);

    // std::cout << "wrench:" << std::endl;

    for (unsigned int step = 0; step < m_steps; ++step) {
        double t = time + step * m_configuration.time_step;

        m_joint_position[step] = m_dynamics->get_joint_position();
        m_end_effector[step] = m_dynamics->get_end_effector_state();
        m_power[step] = m_dynamics->get_power();
        m_energy[step] = m_dynamics->get_tank_energy();

        Vector6d wrench = m_end_effector_wrench_forecast->forecast(t);
        m_end_effector_wrench[step] = wrench;
        // std::cout << "    " << wrench.head<3>().transpose() << std::endl;

        // Simulate the forecast wrench trajectory.
        m_dynamics->add_end_effector_simulated_wrench(wrench);

        // Step the dynamics simulation.
        m_dynamics->step(control, m_configuration.time_step);
    }

    // std::cout << "forecast:" << std::endl;
    // for (auto &x : m_joint_position)
    //     std::cout << "    " << x.head<3>().transpose() << std::endl;

    m_last_forecast = time;
}

} // namespace FrankaRidgeback
