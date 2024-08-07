#pragma once

#include <cmath>

class EnergyTank
{
public:

    inline EnergyTank(double state, double time)
        : m_state(state)
        , m_energy(0.5 * state * state)
    {}

    inline void step(double energy, double dt) {
        m_energy = std::max(0.0, m_energy + energy * dt);
        m_state = std::sqrt(2.0 * m_energy);
    }

    inline double get_state() const {
        return m_state;
    }

    inline double get_energy() const {
        return m_energy;
    }

private:

    double m_state;

    double m_energy;
};
