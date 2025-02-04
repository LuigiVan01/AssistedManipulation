#pragma once

#include <optional>

#include "simulation/simulator.hpp"
#include "simulation/frankaridgeback/actor_dynamics.hpp"
#include "controller/pid.hpp"
#include "controller/mppi.hpp"
#include "controller/energy.hpp"
#include "controller/forecast.hpp"
#include "controller/filterqp.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"
#include "frankaridgeback/objective/track_point.hpp"
#include "frankaridgeback/objective/assisted_manipulation.hpp"

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

        /// Configuration of the actor dynamics.
        SimulatorDynamics::Configuration dynamics;

        /**
         * @brief Configuration of the objective function.
         */
        struct Objective {

            enum Type {
                ASSISTED_MANIPULATION,
                TRACK_POINT
            };

            /// The selected objective.
            Type type;

            /// Configuration for the assisted manipulation objective.
            std::optional<FrankaRidgeback::AssistedManipulation::Configuration> assisted_manipulation;

            /// Configuration for the reach objective.
            std::optional<FrankaRidgeback::TrackPoint::Configuration> track_point;

            // JSON conversion for base simulation objective.
            NLOHMANN_DEFINE_TYPE_INTRUSIVE(
                Objective,
                type, assisted_manipulation, track_point
            )
        };

        /// The objective configuration.
        Objective objective;

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
            mppi, dynamics, objective, forecast, controller_rate,
            controller_substeps, forecast_rate
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
     * 
     * @returns A pointer to the actor on success, or nullptr on failure.
     */
    static std::shared_ptr<Actor> create(
        Configuration configuration,
        Simulator *simulator
    );

    /**
     * @brief Add wrench to the actors end effector.
     * @param wrench The wrench to add in the world frame.
     * @param time The time of the wrench.
     */
    void add_end_effector_wrench(Vector6d wrench, double time);

    /**
     * @brief Get the most recent control applied to the actor dynamics.
     */
    inline const FrankaRidgeback::Control &get_control() const
    {
        return m_control;
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

    /**
     * @brief Get a pointer to the simulated articulated system.
     */
    inline const FrankaRidgeback::Dynamics &get_dynamics() const
    {
        return *m_dynamics->get_dynamics();
    }

    /**
     * @brief Get a pointer to the dynamics forecast if it exists.
     * 
     * @warning May be nullptr.
     * @returns A pointer to the dynamics forecast on success or nullptr if
     * there is no dynamics forecast handle.
     */
    inline DynamicsForecast *get_forecast()
    {
        if (m_forecast)
            return m_forecast.get();
        return nullptr;
    }

private:

    friend class Simulator;

    Actor(
        Configuration &&configuration,
        std::unique_ptr<ActorDynamics> &&dynamics,
        std::unique_ptr<mppi::Trajectory> &&controller,
        std::unique_ptr<DynamicsForecast> &&forecast,
        std::unique_ptr<FilterQP> &&filterqp,
        std::int64_t controller_countdown_max,
        std::int64_t forecast_countdown_max
    );

    /**
     * @brief Calls forecast and update controller, 
     * makes the actor act performing an action in the world 
     * based on the controller output
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

    /// High frequency Safety Filter
    std::unique_ptr<FilterQP> m_qpfilter;

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

using ObjectiveType = Actor::Configuration::Objective::Type;
using DynamicsType = Actor::Type;

} // FrankaRidgeback
