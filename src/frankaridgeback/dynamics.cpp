#include "frankaridgeback/dynamics.hpp"

#include <limits>

namespace FrankaRidgeback {

std::array<std::string, (std::size_t)Frame::_SIZE> FRAME_NAMES {{
    "world_joint",
    "x_base_joint",
    "y_base_joint",
    "pivot_joint",
    "panda_joint1",
    "omni_base_flange",
    "base_link_joint",
    "mid_mount_joint",
    "right_side_cover_link_joint",
    "left_side_cover_link_joint",
    "front_cover_link_joint",
    "rear_cover_link_joint",
    "front_lights_link_joint",
    "rear_lights_link_joint",
    "top_link_joint",
    "axle_joint",
    "imu_joint",
    "ridgeback_sensor_mount_joint",
    "reference_link_joint",
    "arm_mount_joint",
    "panda_joint_franka_mount_link",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
    "panda_finger_joint1",
    "panda_finger_joint2",
    "panda_joint8",
    "panda_hand_joint",
    "panda_grasp_joint"
}};

std::array<std::string, (std::size_t)Link::_SIZE> LINK_NAMES {{
    "omni_base_root_link",
    "x_slider",
    "y_slider",
    "pivot",
    "panda_link1",
    "panda_link2",
    "panda_link3",
    "panda_link4",
    "panda_link5",
    "panda_link6",
    "panda_link7",
    "panda_leftfinger",
    "panda_rightfinger"
}};

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
      , m_joint_power(steps, 0.0)
      , m_external_power(steps, 0.0)
      , m_energy(steps, 0.0)
      , m_end_effector_wrench(steps, Vector6d::Zero())
{}

void DynamicsForecast::forecast(State state, double time)
{
    auto control = Control::Zero();
    m_dynamics->set_state(state, time);

    // std::cout << "state: " << state.position().transpose() << std::endl;
    // std::cout << "wrench:" << std::endl;

    for (unsigned int step = 0; step < m_steps; ++step) {
        double t = time + step * m_configuration.time_step;

        m_joint_position[step] = m_dynamics->get_joint_position();
        m_end_effector[step] = m_dynamics->get_end_effector_state();

        m_joint_power[step] = m_dynamics->get_joint_power();
        m_external_power[step] = m_dynamics->get_external_power();
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
