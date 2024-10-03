#pragma once

#include <memory>
#include <string>

#include "controller/eigen.hpp"
#include "controller/json.hpp"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

/**
 * @brief Raisim simulator.
 */
class Simulator
{
public:

    /**
     * @brief An actor interacts with the simulation.
     */
    class Actor
    {
    public:

        /**
         * @brief Perform an action in the world.
         * @param handle The handle to the simulator to get data from.
         */
        virtual void act(Simulator *simulator) = 0;

        /**
         * @brief Update the state of the actor after acting.
         * @param simulator Pointer to the simulator to update state from.
         */
        virtual void update(Simulator *simulator) = 0;
    };

    /**
     * @brief Simulator configuration.
     */
    struct Configuration {

        // The timestep of the simulation.
        double time_step;

        // The gravitational acceleration.
        Vector3d gravity;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, time_step, gravity)
    };

    /// The default configuration of the simulator.
    static inline const Configuration DEFAULT_CONFIGURATION {
        .time_step = 0.01,
        .gravity = Vector3d(0, 0, 9.81)
    };

    /**
     * @brief Activate the raisim licence.
     */
    static void activate();

    /**
     * @brief Create a new instance of the simulator.
     * @returns A pointer to the simulator on success, or nullptr on failure.
     */
    static std::unique_ptr<Simulator> create(const Configuration &configuration);

    /**
     * @brief Add add an actor to the simulation.
     * @param actor The actor.
     */
    inline void add_actor(const std::shared_ptr<Actor> &actor) {
        m_actors.emplace_back(actor);
    }

    /**
     * @brief Step the simulation.
     */
    void step();

    /**
     * @brief Get a pointer to the simulator world.
     */
    std::shared_ptr<raisim::World> get_world() {
        return m_world;
    }

    /**
     * @brief Get a reference to the raisim server.
     */
    raisim::RaisimServer &get_server() {
        return m_server;
    }

    /**
     * @brief Get the time in the simulation.
     * @returns The time elapsed in seconds.
     */
    double get_time() const {
        return m_world->getWorldTime();
    }

    /**
     * @brief Get the time step of each simulation update.
     * @returns The change in time.
     */
    double get_time_step() const {
        return m_configuration.time_step;
    }

private:

    Simulator(
        const Configuration &configuration,
        std::shared_ptr<raisim::World> &&world
    );

    /// The simulators configuration.
    Configuration m_configuration;

    /// The simulated world.
    std::shared_ptr<raisim::World> m_world;

    /// The server that can be connected to by the raisim viewer.
    raisim::RaisimServer m_server;

    /// The actors in the simulation.
    std::vector<std::shared_ptr<Actor>> m_actors;
};
