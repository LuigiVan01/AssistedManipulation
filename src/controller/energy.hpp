#pragma once

#include <cmath>

class EnergyTank
{
public:

    /**
     * @brief Initialise the energy tank.
     * 
     * @param energy The initial energy in the tank.
     */
    inline EnergyTank(double energy)
        : m_energy(energy)
        , m_state(std::sqrt(2.0 * energy))
    {}

    inline void step(double power, double dt) {
        m_energy = std::max(0.0, m_energy + power * dt);
        m_state = std::sqrt(2.0 * m_energy);
    }

    inline double get_state() const {
        return m_state;
    }

    inline double get_energy() const {
        return m_energy;
    }

private:

    double m_energy;

    double m_state;
};
