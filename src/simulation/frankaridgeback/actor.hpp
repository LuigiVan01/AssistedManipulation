#pragma once

#include <optional>

#include "simulation/simulator.hpp"
#include "simulation/frankaridgeback/actor_dynamics.hpp"
#include "controller/pid.hpp"
#include "controller/mppi.hpp"
#include "controller/energy.hpp"
#include "controller/forecast.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"

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

    /**
     * @brief The type of MPPI dynamics to pass to the mppi trajectory
     * generator.
     */
    enum Type {
        RAISIM = 0,
        PINOCCHIO = 1
    };

    struct Configuration {

        /// Configuration of the actor dynamics.
        SimulatorDynamics::Configuration dynamics;

        /**
         * @brief Configuration structure for the mppi trajectory generator.
         */
        struct MPPI {

            /// Configuration of the mppi controller
            mppi::Configuration configuration;

            /// Configuration of the mppi rollout dynamics.
            SimulatorDynamics::Configuration dynamics;

            // JSON conversion for MPPI configuration.
            NLOHMANN_DEFINE_TYPE_INTRUSIVE(MPPI, configuration, dynamics)
        };

        /// Configuration of the trajectory generator.
        MPPI mppi;

        /**
         * @brief Configuration structure for the dynamics forecast.
         */
        struct Forecast {

            /// Configuration of the dynamics forecast.
            DynamicsForecast::Configuration configuration;

            /// Configuration of the dynamics used for forecast external wrench.
            SimulatorDynamics::Configuration dynamics;

            // JSON conversion for Forecast configuration.
            NLOHMANN_DEFINE_TYPE_INTRUSIVE(Forecast, configuration, dynamics)
        };

        /// The configuration of the forecast, if provided.
        std::optional<Forecast> forecast;

        /// The period of time between the controller updates.
        double controller_rate;

        /// The number of controller updates each update.
        unsigned int controller_substeps;

        /// The period of time between forecast observations.
        double forecast_rate;

        // JSON conversion for franka ridgeback actor configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            dynamics, mppi, forecast, controller_rate, controller_substeps,
            forecast_rate
        )
    };

    // static const Configuration DEFAULT_CONFIGURATION;

    /**
     * @brief Create a franka-ridgeback actor.
     * 
     * Configuration is copied.
     * 
     * @param configuration The configuration of the actor.
     * @param simulator Pointer to the owning simulator.
     * @param dynamics Pointer to the dynamics to use for trajectory generation.
     * @param cost Pointer to the cost to use for trajectory generation.
     * @param filter Pointer to the optional safety filter.
     * 
     * @returns A pointer to the actor on success, or nullptr on failure.
     */
    static std::shared_ptr<Actor> create(
        Configuration configuration,
        Simulator *simulator,
        std::unique_ptr<mppi::Cost> &&cost,
        std::unique_ptr<mppi::Filter> &&filter = nullptr
    );

    /**
     * @brief Add wrench to the actors end effector.
     * @param wrench The wrench to add in the world frame.
     * @param time The time of the wrench.
     */
    void add_end_effector_wrench(Vector6d wrench, double time);

    /**
     * @brief Get a pointer to the simulated articulated system.
     */
    inline const FrankaRidgeback::Dynamics &get_dynamics() const
    {
        return *m_dynamics->get_dynamics();
    }

    /**
     * @brief Get the current robot simulated state.
     */
    inline const FrankaRidgeback::State &get_state() const
    {
        return m_dynamics->get_dynamics()->get_state();
    }

    /**
     * @brief Get the mppi trajectory generator.
     */
    inline const mppi::Trajectory &get_controller() const
    {
        return *m_controller;
    }

private:

    friend class Simulator;

    Actor(
        Configuration &&configuration,
        std::unique_ptr<ActorDynamics> &&dynamics,
        std::unique_ptr<mppi::Trajectory> &&controller,
        std::unique_ptr<DynamicsForecast> &&forecast,
        std::int64_t controller_countdown_max,
        std::int64_t forecast_countdown_max
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

    /// The simulated dynamics.
    std::unique_ptr<ActorDynamics> m_dynamics;

    /// The forecast dynamics.
    std::unique_ptr<DynamicsForecast> m_forecast;

    /// The trajectory generator.
    std::unique_ptr<mppi::Trajectory> m_controller;

    /// Countdown to next trajectory update.
    std::int64_t m_trajectory_countdown;

    /// The value to reset the trajectory countdown after update.
    std::int64_t m_trajectory_countdown_max;

    /// Countdown to next forecast observation update.
    std::int64_t m_forecast_countdown;

    /// The value to reset the forecast countdown after update.
    std::int64_t m_forecast_countdown_max;

    /// The current control action.
    FrankaRidgeback::Control m_control;
};

} // FrankaRidgeback
