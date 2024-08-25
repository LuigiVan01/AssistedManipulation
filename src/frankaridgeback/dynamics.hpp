#pragma once

#include <utility>

#include "controller/mppi.hpp"
#include "controller/kalman.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

/**
 * @brief The dynamics of the model parameters.
 * 
 * Defines how State evolves with Control input.
 */
class Dynamics : public mppi::Dynamics
{
public:

    /// TODO: Add force prediction

    static std::unique_ptr<Dynamics> create();

    inline constexpr int state_dof() override {
        return DoF::STATE;
    }

    inline constexpr int control_dof() override {
        return DoF::CONTROL;
    }

    /**
     * @brief Set the dynamics simulation to a given state.
     * 
     * @param state The system state.
     * @param t The time in the simulation.
     */
    inline void set(const Eigen::VectorXd &state) override {
        m_state = state;
    };

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) override;

    inline Eigen::Ref<Eigen::VectorXd> get() {
        return m_state;
    }

    inline std::unique_ptr<mppi::Dynamics> copy() override {
        Dynamics *dynamics = new Dynamics();
        dynamics->m_state = m_state;
        return std::unique_ptr<Dynamics>(dynamics);
    }

private:

    /**
     * @brief Initialise the dynamics parameters.
     */
    Dynamics();

    /// The current state.
    State m_state;
};

} // namespace FrankaRidgeback
