#include "controller.hpp"

#include <Eigen/Eigen>
#include <cmath>
#include <memory>

// #include "raisim/RaisimServer.hpp"
// #include "raisim/World.hpp"

// raisim::World::setActivationKey(std::getenv("RAISIM_ACTIVATION"));

// auto world = std::make_unique<raisim::World>();
// world->setTimeStep(timestep);
// world->setGravity({0, 0, gravity});

// auto ground = world->addGround();
// ground->setName("ground");
// ground->setAppearance("grid");

// // Add a simulated sphere to the world.
// auto sphere = world->addSphere(0.01, mass);
// sphere->setPosition(initial_position);

// raisim::RaisimServer server {world.get()};
// server.launchServer();

    // Get the ball state.
    // state.head(3) = sphere->getPosition();
    // state.tail(3) = sphere->getLinearVelocity();

    // sphere->setExternalForce(0, control);

    // Simulate.
    // server.integrateWorldThreadSafe();

class Dynamics : public mppi::Dynamics
{
public:

    Dynamics(double mass, double gravity)
        : m_mass(mass)
        , m_gravity(gravity)
        , m_state(6)
    {}

    inline int state_dof() override {
        return 6;
    }

    inline int control_dof() override {
        return 3;
    }

    inline void set(const Eigen::VectorXd &state) override {
        m_state = state;
    };

    inline Eigen::Ref<Eigen::VectorXd> step(
        const Eigen::VectorXd &control,
        double dt
    ) override {
        m_state.tail(3) += (control + Eigen::Vector3d(0, 0, m_gravity)) * dt;
        m_state.head(3) += m_mass * m_state.tail(3) * dt;
        return m_state;
    }

    inline Eigen::Ref<Eigen::VectorXd> get() {
        return m_state;
    }

    double m_mass;
    double m_gravity;
    Eigen::VectorXd m_state;
};

class Cost : public mppi::Cost
{
public:

    Cost(Eigen::Vector3d target)
        : m_target(target)
    {}

    inline constexpr int state_dof() override {
        return 6;
    }

    inline constexpr int control_dof() override {
        return 3;
    }

    inline double get(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        double dt
    ) override {
        double cost = 0.0;

        // auto loglimit = [](double x, double limit, double strength) {
        //     return strength / (limit - x);
        // };

        // double control_norm = control.norm();
        // if (control_norm > 10)
        //     cost += 100 * loglimit(control_norm, 20, 100);

        // Aim for target position.
        cost += 100 * std::pow((state.head(3) - m_target).norm(), 2);

        // Aim for zero velocity.
        cost += std::pow(state.tail(3).norm(), 2);

        // cost = std::pow(state.norm(), 2);

        return cost;
    }

    Eigen::Vector3d m_target;
};

int main()
{
    double mass = 0.1;
    double gravity = -9.81;

    Eigen::Vector3d initial_position = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d initial_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d target_position = Eigen::Vector3d(0.5, 0.5, 0.5);

    Eigen::VectorXd state = Eigen::VectorXd(6, 1).setZero();
    state.head(3) = initial_position;
    state.tail(3) = initial_velocity;

    controller::Configuration configuration {
        .dynamics = std::make_unique<Dynamics>(mass, gravity),
        .cost = std::make_unique<Cost>(target_position),
        .trajectory = {
            .rollouts = 100,
            .keep_best_rollouts = 70,
            .step_size = 0.05,
            .horison = 0.25,
            .gradient_step = 1.0,
            .gradient_minmax = 1.0,
            .cost_scale = 10.0,
            .cost_discount_factor = 0.99,
            .covariance = Eigen::Vector3d(0.1, 0.1, 0.1).asDiagonal(),
            .control_default_last = true,
            .control_default_value = Eigen::Vector3d::Zero()
        },
        .initial_state = state,
    };

    auto controller = controller::Controller::create(std::move(configuration));
    if (!controller) {
        std::cerr << "failed to create controller" << std::endl;
        return 1;
    }

    int steps = (int)(configuration.trajectory.horison / configuration.trajectory.step_size);
    double timestep = 0.005;

    Dynamics sim {mass, gravity};
    sim.set(state);

    // The current control action.
    Eigen::Vector3d control;

    // Time in the simulation.
    double time = 0.0;

    for (;;) {

        state = sim.get();
        std::cout << state.transpose() << std::endl;

        // Update the trajectory.
        controller->update(state, time);

        // Apply the control trajectory to the sphere.
        for (std::size_t i = 0; i < steps; i++) {

            // Apply the trajectory forces to the sphere.
            controller->get(control, time);

            // Simulate.
            sim.step(control, timestep);
            time += timestep;
        }
    }
}
