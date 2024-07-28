// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
#include <cstdlib>
#include <filesystem>

#include "simulation/simulator.hpp"
#include "simulation/actors/frankaridgeback.hpp"
#include "simulation/actors/circle.hpp"
#include "frankaridgeback/dynamics.hpp"

int main(int /* argc */, char*[])
{
    std::unique_ptr<Simulator> simulator = Simulator::create({
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    });

    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return 1;
    }

    auto cwd = std::filesystem::current_path();
    std::string urdf = (cwd / "model/robot.urdf").string();

    FrankaRidgebackActor::Configuration configuration {
        .controller = {
            .dynamics = FrankaRidgeback::Dynamics::create(),
            .cost = TrackPoint::create({
                .point = Eigen::Vector3d(1.0, 1.0, 1.0),
                .model = {
                    .filename = urdf,
                    .end_effector_frame = "panda_grasp"
                }
            }),
            .trajectory = {
                .rollouts = 20,
                .keep_best_rollouts = 10,
                .time_step = 0.1,
                .horison = 1.0,
                .gradient_step = 1.0,
                .cost_scale = 10.0,
                .cost_discount_factor = 1.0,
                .covariance = FrankaRidgeback::Control{
                    0.0, 0.0, 0.2, // base
                    10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
                    0.0, 0.0 // gripper
                }.asDiagonal(),
                .control_bound = false,
                .control_min = FrankaRidgeback::Control{
                    -0.2, -0.2, -0.2, // base
                    -5.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, //arm
                    -0.05, -0.05 // gripper
                },
                .control_max = FrankaRidgeback::Control{
                    0.2, 0.2, 0.2, // base
                    5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, // arm
                    0.05, 0.05 // gripper
                },
                .control_default = FrankaRidgeback::Control::Zero(),
                .filter = std::nullopt,
                .threads = 12
            },
            .initial_state = FrankaRidgeback::State::Zero(),
        },
        .controller_rate = 0.3,
        .controller_substeps = 10,
        .urdf_filename = urdf,
        .end_effector_frame = "panda_grasp_joint",
        .initial_state = FrankaRidgeback::State::Zero(),
        .proportional_gain = FrankaRidgeback::Control{
            0.0, 0.0, 0.0, // base
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // arm
            100.0, 100.0 //gripper
        },
        .differential_gain = FrankaRidgeback::Control{
            1000.0, 1000.0, 1.0, // base
            10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
            50.0, 50.0 // gripper
        },
    };

    if (!configuration.controller.cost) {
        std::cerr << "failed to create controller cost" << std::endl;
        return 1;
    }

    if (!configuration.controller.dynamics) {
        std::cerr << "failed to create controller dynamics" << std::endl;
        return 1;
    }

    auto robot = FrankaRidgebackActor::create(
        std::move(configuration),
        simulator.get()
    );

    if (!robot) {
        std::cerr << "failed to create FrankaRidgebackActor actor" << std::endl;
        return 1;
    }

    simulator->add_actor(robot);

    for (;;) {
        raisim::TimedLoop(simulator->get_time_step() * 1e6);
        simulator->step();
    }
}
