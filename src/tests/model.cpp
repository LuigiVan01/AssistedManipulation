#include <filesystem>
#include <iostream>

#include "simulator.hpp"
#include "dynamics.hpp"

// std::cout << "creating simulator" << std::endl;
// Simulator::Configuration simulator {
//     .urdf_filename = urdf,
//     .timestep = 0.005,
//     .gravity = {0.0, 0.0, 9.81},
//     .initial_state = configuration.initial_state,
//     .proportional_gain = FrankaRidgeback::Control::Zero(),
//     .differential_gain = FrankaRidgeback::Control::Zero()
// };

// std::unique_ptr<Simulator> simulator = Simulator::create(simulator);

int main()
{
    using namespace FrankaRidgeback;

    auto cwd = std::filesystem::current_path();
    std::string urdf = (cwd / "model/robot.urdf").string();

    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;

    State state = State::Zero();

    auto dynamics = Dynamics::create();
    auto model = Model::create(urdf);

    // Get the end effector position.
    state = State::Zero();
    model->set(state);
    std::tie(pos, ori) = model->end_effector();
    std::cout << "x to 0.0:" << std::endl;
    std::cout << "    end effector: " << pos.transpose() << std::endl;

    // Get the end effector position.
    state(0) = 10.0;
    model->set(state);
    std::tie(pos, ori) = model->end_effector();
    std::cout << "x to 10.0:" << std::endl;
    std::cout << "    end effector: " << pos.transpose() << std::endl;

    std::cout << "vx to 1 after 1 second:" << std::endl;
    Control control = Control::Zero();
    control(0) = 1.0;
    dynamics->set(state);
    dynamics->step(control, 1.0);
    std::cout << "    state: " << pos.transpose() << std::endl;
    model->set(dynamics->get());
    std::tie(pos, ori) = model->end_effector();
    std::cout << "    end effector: " << pos.transpose() << std::endl;

    return 0;
}
