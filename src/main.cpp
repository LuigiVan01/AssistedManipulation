// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
#include <cstdlib>
#include <filesystem>

#include "simulator.hpp"
#include "controller.hpp"
#include "cost.hpp"
#include "dynamics.hpp"

int main(int /* argc */, char*[])
{
    auto cwd = std::filesystem::current_path();
    std::string urdf = (cwd / "model/robot.urdf").string();

    Eigen::VectorXd stddev(12, 1);
    stddev << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    controller::Configuration configuration {
        .dynamics = FrankaRidgeback::Dynamics::create(),
        .cost = Cost::create(urdf),
        .trajectory = {
            .rollouts = 10,
            .keep_best_rollouts = 5,
            .step_size = 0.05,
            .horison = 0.25,
            .gradient_step = 1.0,
            .gradient_minmax = 10.0,
            .cost_scale = 1.0,
            .cost_discount_factor = 0.95,
            .covariance = stddev.asDiagonal(),
            .control_default_last = true,
            .control_default_value = FrankaRidgeback::Control::Zero()
        },
        .initial_state = FrankaRidgeback::State::Zero(),
    };

    auto controller = controller::Controller::create(std::move(configuration));
    if (!controller) {
        std::cerr << "failed to create controller" << std::endl;
        return 1;
    }

    std::cout << "creating simulator" << std::endl;
    Simulator::Configuration simulator {
        .urdf_filename = urdf,
        .timestep = 0.005,
        .gravity = {0.0, 0.0, 9.81},
        .initial_state = configuration.initial_state,
        .proportional_gain = FrankaRidgeback::Control::Zero(),
        .differential_gain = FrankaRidgeback::Control::Zero()
    };

    simulator.proportional_gain.base_velocity() << 0.0, 0.0;
    simulator.proportional_gain.base_angular_velocity() << 0.0;
    simulator.proportional_gain.arm_torque() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    simulator.proportional_gain.gripper_position() << 100.0, 100.0;

    simulator.differential_gain.base_velocity() << 1000.0, 1000.0;
    simulator.differential_gain.base_angular_velocity() << 1.0;
    simulator.differential_gain.arm_torque() << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    simulator.differential_gain.gripper_position() << 50.0, 50.0;

    std::unique_ptr<Simulator> sim = Simulator::create(simulator);
    if (!sim) {
        std::cerr << "failed to create simulator" << std::endl;
        return 1;
    }

    int steps = (int)(configuration.trajectory.horison / configuration.trajectory.step_size);

    FrankaRidgeback::Control control;

    for (;;) {
        Eigen::VectorXd state = sim->state();
        controller->update(state, sim->time());
        for (std::size_t i = 0; i < steps; i++) {
            raisim::TimedLoop(simulator.timestep * 1e6);
            controller->get(control, sim->time());
            sim->step(control);
        }
    }
}
