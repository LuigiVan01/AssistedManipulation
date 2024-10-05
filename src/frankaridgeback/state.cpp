#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

State make_state(Preset preset)
{
    State state = State::Zero();

    switch (preset)
    {
        case Preset::HUDDLED_10J: {
            state.position()
                << 0.0, 0.0, 0.0, 0.0, 0.628, 0.0, -0.628, 0.0, 1.88, 0.0, 0.00, 0.00;
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
