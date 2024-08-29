#pragma once

#include <utility>

#include "controller/energy.hpp"
#include "controller/json.hpp"
#include "controller/kalman.hpp"
#include "controller/mppi.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"
#include "frankaridgeback/model.hpp"

namespace FrankaRidgeback {

/**
 * @brief The dynamics of the model parameters.
 * 
 * Defines how State evolves with Control input.
 */
class Dynamics : public mppi::Dynamics
{
public:

    enum ForceEstimationStrategy {
        LINEAR = 0,
        KALMAN = 1
    };

    struct Configuration {

        /// The configuration of the model.
        FrankaRidgeback::Model::Configuration model;

        /// The initial energy in the energy tank.
        double energy = 0.0;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, model, energy);
    };

    /**
     * @brief Create a new franka ridgeback dynamics object.
     * @returns A pointer to the dynamics on success or nullptr on failure.
     */
    static std::unique_ptr<Dynamics> create(const Configuration &configuration);

    /**
     * @brief Get the number of degrees of freedom for the frankaridgeback
     * state.
     */
    inline constexpr int state_dof() override {
        return DoF::STATE;
    }

    /**
     * @brief Get the number of degrees of freedom for the frankaridgeback
     * control.
     */
    inline constexpr int control_dof() override {
        return DoF::CONTROL;
    }

    /**
     * @brief Get the dynamics state.
     */
    inline Eigen::Ref<Eigen::VectorXd> get() {
        return m_state;
    }

    /**
     * @brief Set the dynamics simulation to a given state.
     * @param state The system state.
     */
    inline void set(const Eigen::VectorXd &state) override {
        m_model->set(state);
        m_state = state;
    };

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) override;

    /**
     * @brief Get a pointer to the dynamics model.
     */
    inline FrankaRidgeback::Model *get_model() {
        return m_model.get();
    }

    /**
     * @brief Copy the dynamics.
     */
    inline std::unique_ptr<mppi::Dynamics> copy() override {
        return std::unique_ptr<Dynamics>(
            new Dynamics(m_energy_tank.get_energy(), std::move(m_model->copy()))
        );
    }

private:

    /**
     * @brief Initialise the dynamics parameters.
     * 
     * @param energy The initial energy in the energy tank.
     * @param model The robot model to update.
     */
    Dynamics(
        double energy,
        std::unique_ptr<FrankaRidgeback::Model> &&model
    );

    /// Pointer to the model to calculate proximity to the point.
    std::unique_ptr<FrankaRidgeback::Model> m_model;

    /// The energy tank.
    EnergyTank m_energy_tank;

    /// The current state.
    State m_state;
};

} // namespace FrankaRidgeback
