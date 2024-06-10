// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
#include <cstdlib>
#include <filesystem>

#include "simulator.hpp"
#include "cost.hpp"
#include "dynamics.hpp"
#include "mppi.hpp"

int main(int /* argc */, char*[])
{
    auto cwd = std::filesystem::current_path();

    std::string urdf = (cwd / "model/robot.urdf").string();

    // Create the controller.
    mppi::Configuration controller_configuration {
        .rollouts = 10,
        .rollouts_cached = 0,
        .step_size = 0.05,
        .horison = 1.0,
        .gradient_step = 1.0,
        .gradient_minmax = 10000.0,
        .cost_scale = 1.0,
        .cost_discount_factor = 0.9,
        .control_default_last = true,
        .control_default_value = FrankaRidgeback::Dynamics::Control::Zero(),
    };

    // Set the initial state.
    auto initial_state = FrankaRidgeback::State::Zero().eval();

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
    auto trajectory = mppi::Trajectory<FrankaRidgeback::Dynamics, Cost>::create(
        dynamics,
        cost,
        initial_state,
        controller_configuration
    );

    if (!trajectory) {
        std::cerr << "failed to create trajectory" << std::endl;
        return 1;
    }

    Simulator::Configuration simulator_configuation {
        .urdf_filename = urdf,
        .timestep = 0.005,
        .gravity = 9.81,
        .initial_state = initial_state
    };

    std::unique_ptr<Simulator> sim = Simulator::create(simulator_configuation);
    if (!sim) {
        std::cerr << "failed to create simulator" << std::endl;
        return 1;
    }

    for (;;) {
        trajectory->update(sim->state(), sim->time());

        for (std::size_t i = 0; i < 100; i++) {
            FrankaRidgeback::Control control = trajectory->get(sim->time());
            sim->step(control);
        }
    }
}
