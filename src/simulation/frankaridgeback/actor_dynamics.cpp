#include "simulation/frankaridgeback/actor_dynamics.hpp"

namespace FrankaRidgeback {

std::unique_ptr<RaisimActorDynamics> RaisimActorDynamics::create(
    RaisimDynamics::Configuration configuration,
    Simulator *simulator
) {
    // Use the passed in raisim world, rather than instantiating its own.
    configuration.simulator = std::nullopt;

    // Create the raisim dynamics to simulate the robot with, using the existing
    // simulated world to instantiate the model in. The model does not forecast.
    auto dynamics = FrankaRidgeback::RaisimDynamics::create(
        configuration,
        nullptr,
        simulator
    );
    if (!dynamics)
        return nullptr;

    return std::unique_ptr<RaisimActorDynamics>(
        new RaisimActorDynamics(simulator, std::move(dynamics))
    );
}

std::unique_ptr<PinocchioActorDynamics> PinocchioActorDynamics::create(
    const PinocchioDynamics::Configuration &configuration,
    Simulator *simulator
) {
    // The simulated model does not forecast.
    auto pinocchio = PinocchioDynamics::create(configuration);
    if (!pinocchio)
        return nullptr;

    // Add a visual of the pinocchio controlled system.
    auto visual = simulator->get_server().addVisualArticulatedSystem(
        "pinocchio",
        pinocchio->get_configuration().filename
    );

    return std::unique_ptr<PinocchioActorDynamics>(
        new PinocchioActorDynamics(simulator, std::move(pinocchio), visual)
    );
}

std::unique_ptr<ActorDynamics> ActorDynamics::create(
    const SimulatorDynamics::Configuration &configuration,
    Simulator *simulator
) {
    using Type = SimulatorDynamics::Configuration::Type;

    std::unique_ptr<ActorDynamics> model = nullptr;

    if (configuration.type == Type::RAISIM) {
        if (!configuration.raisim) {
            std::cerr << "selected raisim dynamics for model without configuration" << std::endl;
            return nullptr;
        }

        model = RaisimActorDynamics::create(*configuration.raisim, simulator);
        if (!model) {
            std::cerr << "failed to create raisim dynamics model model" << std::endl;
            return nullptr;
        }
    }
    else if (configuration.type == Type::PINOCCHIO) {
        if (!configuration.pinocchio) {
            std::cerr << "selected pinocchio dynamics for model without configuration" << std::endl;
            return nullptr;
        }

        model = PinocchioActorDynamics::create(*configuration.pinocchio, simulator);
        if (!model) {
            std::cerr << "failed to create pinocchio dynamics model model" << std::endl;
            return nullptr;
        }
    }
    else {
        std::cerr << "unknown dynamics model model type " << configuration.type << std::endl;
        return nullptr;
    }

    return model;
}

} // namspace FrankaRidgeback
