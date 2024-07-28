#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>

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
        Eigen::Vector3d gravity;
    };

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
    raisim::World &get_world() {
        return *m_world;
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
        return m_time;
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
        std::unique_ptr<raisim::World> &&world
    );

    /// The simulators configuration.
    Configuration m_configuration;

    /// The simulated world.
    std::unique_ptr<raisim::World> m_world;

    /// The server that can be connected to by the raisim viewer.
    raisim::RaisimServer m_server;

    /// The actors in the simulation.
    std::vector<std::shared_ptr<Actor>> m_actors;

    /// The time in the simulation.
    double m_time;
};
