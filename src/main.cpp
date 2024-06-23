// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
#include <cstdlib>
#include <filesystem>

#include "simulator.hpp"
#include "cost.hpp"
#include "dynamics.hpp"

int main(int /* argc */, char*[])
{
    auto cwd = std::filesystem::current_path();

    std::string urdf = (cwd / "model/robot.urdf").string();

    // Create the controller.
    mppi::Configuration controller_configuration {
        .rollouts = 100,
        .rollouts_cached = 0,
        .step_size = 0.05,
        .horison = 0.25,
        .gradient_step = 1.0,
        .gradient_minmax = 10.0,
        .cost_scale = 1.0,
        .cost_discount_factor = 0.95,
        .control_default_last = true,
        .control_default_value = FrankaRidgeback::Control::Zero(),
    };

    // Set the initial state.
    FrankaRidgeback::State initial_state = FrankaRidgeback::State::Zero();
    initial_state.base_velocity() << 0.0, 0.0;

    std::cout << "creating dynamics" << std::endl;
    auto dynamics = FrankaRidgeback::Dynamics::create();
    if (!dynamics) {
        std::cerr << "failed to create dynamics" << std::endl;
        return 1;
    }

    std::cout << "creating cost" << std::endl;
    auto cost = Cost::create(urdf);
    if (!cost) {
        std::cerr << "failed to create cost" << std::endl;
        return 1;
    }

    std::cout << "creating trajectory" << std::endl;
    auto trajectory = mppi::Trajectory::create(
        dynamics.get(),
        cost.get(),
        controller_configuration,
        initial_state
    );

    if (!trajectory) {
        std::cerr << "failed to create trajectory" << std::endl;
        return 1;
    }

    std::cout << "creating simulator" << std::endl;
    Simulator::Configuration simulator {
        .urdf_filename = urdf,
        .timestep = 0.005,
        .gravity = {0.0, 0.0, 9.81},
        .initial_state = initial_state,
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

    int steps = (int)(controller_configuration.horison / controller_configuration.step_size);

    for (;;) {
        trajectory->update(sim->state(), sim->time());
        for (std::size_t i = 0; i < steps; i++) {
            raisim::TimedLoop(simulator.timestep * 1e6);
            FrankaRidgeback::Control control = trajectory->get(sim->time());
            sim->step(control);
        }
    }
}
