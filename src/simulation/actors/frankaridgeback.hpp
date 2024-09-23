#pragma once

#include <optional>

#include "simulation/simulator.hpp"
#include "simulation/raisim_dynamics.hpp"
#include "controller/pid.hpp"
#include "controller/mppi.hpp"
#include "controller/energy.hpp"
#include "controller/forecast.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"
#include "frankaridgeback/pinocchio_dynamics.hpp"

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

        /// The raisim simulated dynamics.
        RaisimDynamics::Configuration simulated_dynamics;

        /// The selected type of pinocchio dynamics.
        Type mppi_dynamics_type;

        /// The mppi dynamics configuration for pinocchio if selected.
        std::optional<RaisimDynamics::Configuration> mppi_dynamics_raisim;

        /// The mppi dynamics configuration for raisim if selected.
        std::optional<PinocchioDynamics::Configuration> mppi_dynamics_pinocchio;

        /// Configuration of the mppi trajectory generator.
        mppi::Configuration mppi;

        /// The period of time between the controller updates.
        double controller_rate;

        /// The number of controller updates each update.
        unsigned int controller_substeps;

        // JSON conversion for franka ridgeback actor configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            simulated_dynamics, mppi_dynamics_type, mppi_dynamics_raisim,
            mppi_dynamics_pinocchio, mppi, controller_rate, controller_substeps
        )
    };

    /**
     * @brief Create a franka-ridgeback actor.
     * 
     * Configuration is copied.
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
        Configuration configuration,
        Simulator *simulator,
        std::unique_ptr<mppi::Cost> &&cost,
        std::unique_ptr<Forecast::Handle> &&force_forecast = nullptr,
        std::unique_ptr<mppi::Filter> &&filter = nullptr
    );

    /**
     * @brief Add wrench to the actors end effector.
     * @param wrench The wrench to add, in the world frame.
     */
    inline void add_end_effector_wrench(Vector6d wrench)
    {
        m_dynamics->add_end_effector_true_wrench(wrench);
    }

    /**
     * @brief Get a pointer to the simulated articulated system.
     */
    inline RaisimDynamics *get_dynamics()
    {
        return m_dynamics.get();
    }

    /**
     * @brief Get the current robot simulated state.
     */
    inline const FrankaRidgeback::State &get_state() const
    {
        return m_dynamics->get_state();
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
        std::unique_ptr<RaisimDynamics> &&dynamics,
        std::unique_ptr<mppi::Trajectory> &&controller,
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

    /// The simulated dynamics.
    std::unique_ptr<RaisimDynamics> m_dynamics;

    /// The trajectory generator.
    std::unique_ptr<mppi::Trajectory> m_controller;

    /// Countdown to next trajectory update.
    std::int64_t m_trajectory_countdown;

    /// The value to reset the countdown after update.
    std::int64_t m_trajectory_countdown_max;

    /// The current control action.
    FrankaRidgeback::Control m_control;
};

} // FrankaRidgeback
