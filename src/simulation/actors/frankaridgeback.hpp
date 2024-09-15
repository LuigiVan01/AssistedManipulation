#pragma once

#include "controller/pid.hpp"
#include "controller/mppi.hpp"
#include "controller/energy.hpp"
#include "controller/forecast.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"
#include "simulation/simulator.hpp"
#include "simulation/raisim_dynamics.hpp"

namespace FrankaRidgeback {

/**
 * @brief 
 * 
 * Note that the simulator uses the PD controller with the feed-forward torque
 * control method. This is because the base and the gripper use proportional
 * derivative control for their joints, whereas the arm uses feed-forward torque
 * commands.
 * 
 * @BUG: It is not possible to disable the PD control of the arm joints,
 * including setting the PD gains of the arm to zero. Therefore the torque
 * commands are also opposing the PD controller.
 */
class Actor final : public Simulator::Actor
{
public:

    struct Configuration {

        RaisimDynamics::Configuration dynamics;

        /// Configuration of the mppi trajectory generator.
        mppi::Configuration mppi;

        /// The period of time between the controller updates.
        double controller_rate;

        /// The number of controller updates each update.
        unsigned int controller_substeps;

        // JSON conversion for franka ridgeback actor configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            dynamics, mppi, controller_rate, controller_substeps
        )
    };

    /**
     * @brief Create a franka-ridgeback actor.
     * 
     * @param configuration The configuration of the actor.
     * @param simulator Pointer to the owning simulator.
     * @param dynamics Pointer to the dynamics to use for trajectory generation.
     * @param cost Pointer to the cost to use for trajectory generation.
     * @param force_forecast Pointer to the optional force prediction algorithm.
     * @param filter Pointer to the optional safety filter.
     * @returns A pointer to the actor on success, or nullptr on failure.
     */
    static std::shared_ptr<Actor> create(
        const Configuration &configuration,
        Simulator *simulator,
        std::unique_ptr<mppi::Cost> &&cost,
        std::unique_ptr<Forecast::Handle> &&force_forecast = nullptr,
        std::unique_ptr<mppi::Filter> &&filter = nullptr
    );

    /**
     * @brief Get a pointer to the simulated articulated system.
     */
    inline RaisimDynamics *get_dynamics() {
        return m_dynamics.get();
    }

    /**
     * @brief Get the current robot simulated state.
     */
    inline const FrankaRidgeback::State &get_state() const {
        return m_state;
    }

    /**
     * @brief Get the mppi trajectory generator.
     */
    inline const mppi::Trajectory &get_trajectory() const {
        return *m_trajectory;
    }

private:

    friend class Simulator;

    Actor(
        const Configuration &configuration,
        std::unique_ptr<RaisimDynamics> &&dynamics,
        std::unique_ptr<mppi::Trajectory> &&controller,
        raisim::ArticulatedSystemVisual *visual,
        std::int64_t controller_countdown_max
    );

    /**
     * @brief Perform an action in the world.
     * @param handle The handle to the simulator to get data from.
     */
    void act(Simulator *simulator) override;

    /**
     * @brief Update the state of the frankaridgeback actor after acting.
     * @param simulator Pointer to the simulator to update state from.
     */
    void update(Simulator *simulator) override;

    /// The configuration of the actor.
    Configuration m_configuration;

    std::unique_ptr<RaisimDynamics> m_dynamics;

    /// The trajectory generator.
    std::unique_ptr<mppi::Trajectory> m_trajectory;

    raisim::ArticulatedSystemVisual *m_visual;

    /// Countdown to next trajectory update.
    std::int64_t m_trajectory_countdown;

    /// The value to reset the countdown after update.
    std::int64_t m_trajectory_countdown_max;

    /// The simulated state.
    FrankaRidgeback::State m_state;

    /// The current control action.
    FrankaRidgeback::Control m_control;
};

} // FrankaRidgeback
