#include "simulator.hpp"

void Simulator::activate()
{
    std::string key = std::getenv("RAISIM_ACTIVATION");
    if (key.empty())
        key = "activation.raisim";

    raisim::World::setActivationKey(key.c_str());
}

std::unique_ptr<Simulator> Simulator::create(const Configuration &configuration)
{
    activate();

    auto world = std::make_unique<raisim::World>();
    world->setTimeStep(configuration.time_step);
    world->setGravity(configuration.gravity);

    auto ground = world->addGround();
    ground->setName("ground");
    ground->setAppearance("grid");

    return std::unique_ptr<Simulator>(
        new Simulator(configuration, std::move(world))
    );
}

Simulator::Simulator(
    const Configuration &configuration,
    std::unique_ptr<raisim::World> &&world
) : m_configuration(configuration)
  , m_time(0.0)
  , m_world(std::move(world))
  , m_server(m_world.get())
{
    m_server.launchServer();
}

void Simulator::step()
{
    // Perform all actor actions.
    for (auto &actor : m_actors) {
        actor->act(this);
    }

    // Simulate!
    m_server.integrateWorldThreadSafe();
    m_time += m_configuration.time_step;

    // Update actor states.
    for (auto &actor : m_actors) {
        actor->update(this);
    }
}
