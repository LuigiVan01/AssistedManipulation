#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

State make_state(Preset preset)
{
    State state = State::Zero();

    switch (preset)
    {
        case Preset::ZERO: {
            return State::Zero();
        }
        case Preset::HUDDLED: {
            state.position()          //  1      2    3        4    5  6
                << 0.2, 0.2, M_PI/4, 0.0, M_PI/5, 0.0, -M_PI/2, 0.0, 2, M_PI/4, 0.025, 0.025;
            return state;
        }
        case Preset::BEHIND: {
            state.position()    //  1   2        3        4    5  6
                << 0.20, 0.20, M_PI/4, M_PI, 1.2, 0.0, -2, 0, M_PI/2, M_PI/4, 0.025, 0.025;
            return state;
        }
        case Preset::BELOW: {
            state.position()         //  1    2    3   4  5  6
                << 0.20, 0.20, M_PI/4, 0.0, 1.2, 0.0, -2, 0, M_PI, M_PI/4, 0.025, 0.025;
            return state;
        }
        case Preset::REACH: {
            state.position()         //  1    2    3   4  5  6
                << 0.20, 0.20, M_PI/4, 0.0, 1.5, 0.0, 0, 0, M_PI, M_PI/4, 0.025, 0.025;
            return state;
        }
        case Preset::JOINT_LIMIT: {
            state.position()          //  1      2    3        4    5  6
                << 0.20, 0.20, M_PI/4, 0.0, M_PI/5, 0.0, -M_PI/2, 0.0, -0.2, M_PI/4, 0.025, 0.025;
            return state;
        }
        case Preset::SELF_COLLISION: {
            state.position()          //  1      2    3        4    5  6
                << 0.20, 0.20, M_PI/4, 0.0, M_PI/3, 0.0, -6*M_PI/8, 0.0, 2, M_PI/4, 0.025, 0.025;
            return state;
        }
        default: {
            return State::Zero();
        }
    }
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
