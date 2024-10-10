#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

State make_state(Preset preset)
{
    State state = State::Zero();

    switch (preset)
    {
        case Preset::HUDDLED_10J: {
            state.position()          //  1      2    3        4    5  6
                << 0.20, 0.20, M_PI/4, 0.0, M_PI/5, 0.0, -M_PI/2, 0.0, 4, M_PI/4, 0.025, 0.025;
            state.available_energy().setConstant(10);
            return state;
        }
    }

    return State::Zero();
}

State make_state(
    VectorXd position,
    VectorXd velocity,
    Vector6d wrench,
    double energy
) {
    State state;
    state.position() = position;
    state.velocity() = velocity;
    state.end_effector_wrench() = wrench;
    state.available_energy().setConstant(energy);
    return state;
}

} // namespace FrankaRidgeback
