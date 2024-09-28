#pragma once

#include "simulation/simulator.hpp"
#include "frankaridgeback/pinocchio_dynamics.hpp"
#include "simulation/raisim_dynamics.hpp"

namespace FrankaRidgeback {

/**
 * @brief An interface used by the frankaridgeback actor to use to simulate
 * dynamics.
 * 
 * Enables the usage of non-raisim simulation of the actor dynamics.
 */
class SimulatorAdaptor
{
public:

    enum Type {
        RAISIM = 0,
        PINOCCHIO = 1
    };

    struct Configuration;

    virtual ~SimulatorAdaptor() = default;

protected:

    SimulatorAdaptor(Simulator *simulator)
        : m_simulator(simulator)
    {}

    // Only the FrankaRidgeback::Actor class can create adaptors.
    friend class Actor;

    /**
     * @brief Create a new simulator adaptor.
     * 
     * @param configuration The configuration of the adaptor.
     * @param simulator The simulator to which the adaptor belongs.
     */
    static std::unique_ptr<SimulatorAdaptor> create(
        const Configuration &configuration,
        Simulator *simulator,
        std::unique_ptr<Forecast::Handle> &&wrench_forecast
    );

    /**
     * @brief Get a pointer to the underlying dynamics.
     */
    virtual FrankaRidgeback::Dynamics *get_dynamics() = 0;

    /**
     * @brief Perform any simulation actions before the simulator has stepped.
     * 
     * @param control The current control action of the simulated dynamics.
     * @param dt The subsequent change in simulated time.
     */
    virtual void act(VectorXd control, double dt) = 0;

    /**
     * @brief Perform an update operations after the simulator has stepped.
     */
    virtual void update() = 0;

    /// Pointer to the simulator.
    Simulator *m_simulator;
};

/**
 * @brief Trivial adaptor for the already raisim implemented frankaridgeback
 * dynamics.
 */
class RaisimAdapator : public SimulatorAdaptor
{
public:

    inline static std::unique_ptr<RaisimAdapator> create(
        RaisimDynamics::Configuration configuration,
        Simulator *simulator,
        std::unique_ptr<Forecast::Handle> &&wrench_forecast
    ) {
        // Use the passed in raisim world, rather than instantiating its own.
        configuration.simulator = std::nullopt;

        // Create the raisim dynamics to simulate the robot with.
        auto dynamics = FrankaRidgeback::RaisimDynamics::create(
            configuration,
            std::move(wrench_forecast),
            simulator->get_world()
        );

        if (!dynamics)
            return nullptr;

        return std::unique_ptr<RaisimAdapator>(
            new RaisimAdapator(simulator, std::move(dynamics))
        );
    }

    /**
     * @brief Get a pointer to the underlying dynamics.
     */
    inline FrankaRidgeback::Dynamics *get_dynamics() override
    {
        return m_dynamics.get();
    }

    /**
     * @brief Perform any simulation actions before the simulator has stepped.
     * 
     * @param control The current control action of the simulated dynamics.
     * @param dt The subsequent change in simulated time.
     */
    inline void act(VectorXd control, double /* dt */) override
    {
        m_dynamics->act(control);
    }

    /**
     * @brief Perform an update operations after the simulator has stepped.
     */
    inline void update() override
    {
        m_dynamics->update();
    }

private:

    RaisimAdapator(
        Simulator *simulator,
        std::unique_ptr<RaisimDynamics> &&dynamics
      ) : SimulatorAdaptor(simulator)
        , m_dynamics(std::move(dynamics))
    {}

    std::unique_ptr<RaisimDynamics> m_dynamics;
};

/**
 * @brief Provides pinocchio dynamics visualisation with raisim.
 * 
 * @todo Refactor so not to copy all the derived functions.
 */
class PinocchioAdaptor : public SimulatorAdaptor
{
public:

    /**
     * @brief Create a new raisim pinocchio dynamics adaptor.
     * 
     * @param configuration The configuration of the pinocchio dynamics.
     * @param simulator Pointer to the owning simulator.
     */
    inline static std::unique_ptr<PinocchioAdaptor> create(
        const PinocchioDynamics::Configuration &configuration,
        Simulator *simulator,
        std::unique_ptr<Forecast::Handle> &&wrench_forecast
    ) {
        auto pinocchio = PinocchioDynamics::create(configuration);
        if (!pinocchio)
            return nullptr;

        auto visual = simulator->get_server().addVisualArticulatedSystem(
            "pinocchio",
            configuration.filename
        );

        return std::unique_ptr<PinocchioAdaptor>(
            new PinocchioAdaptor(simulator, std::move(pinocchio), visual)
        );
    }

    /**
     * @brief Get a pointer to the underlying dynamics.
     */
    inline FrankaRidgeback::Dynamics *get_dynamics() override
    {
        return m_dynamics.get();
    }

    /**
     * @brief Perform any simulation actions before the simulator has stepped.
     * 
     * @param control The current control action of the simulated dynamics.
     * @param dt The subsequent change in simulated time.
     */
    inline void act(VectorXd control, double dt) override
    {
        FrankaRidgeback::State state = m_dynamics->step(control, dt);
        m_visual->setGeneralizedCoordinate(state.position());
    }

    /**
     * @brief Perform an update operations after the simulator has stepped.
     */
    inline void update() override {};

    inline ~PinocchioAdaptor()
    {
        m_simulator->get_server().removeVisualArticulatedSystem(m_visual);
    }

private:

    /**
     * @brief Initialise the pinocchio simulator adaptor.
     * 
     * @param pinocchio The dynamics to simulate with.
     * @param visual The raisim visual to use.
     */
    PinocchioAdaptor(
        Simulator *simulator,
        std::unique_ptr<PinocchioDynamics> &&pinocchio,
        raisim::ArticulatedSystemVisual *visual
      ) : SimulatorAdaptor(simulator)
        , m_dynamics(std::move(pinocchio))
        , m_visual(visual)
    {}

    /// The pinocchio dynamics.
    std::unique_ptr<PinocchioDynamics> m_dynamics;

    /// Visualisation of the pinocchio dynamics.
    raisim::ArticulatedSystemVisual *m_visual;
};

struct SimulatorAdaptor::Configuration {

    /// The type of configured adaptor.
    Type type;

    /// The configuration of the raisim dynamics adaptor if selected.
    std::optional<RaisimDynamics::Configuration> raisim;

    /// The configuration of the pinocchio dynamics adaptor if selected.
    std::optional<PinocchioDynamics::Configuration> pinocchio;

    // JSON conversion for simulator adaptor configuration.
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        SimulatorAdaptor::Configuration,
        type, raisim, pinocchio
    )
};

inline std::unique_ptr<SimulatorAdaptor> SimulatorAdaptor::create(
    const Configuration &configuration,
    Simulator *simulator,
    std::unique_ptr<Forecast::Handle> &&wrench_forecast
) {
    std::unique_ptr<SimulatorAdaptor> adaptor = nullptr;

    if (configuration.type == Type::RAISIM) {
        if (!configuration.raisim) {
            std::cerr << "selected raisim dynamics actor without configuration" << std::endl;
            return nullptr;
        }

        adaptor = RaisimAdapator::create(
            *configuration.raisim,
            simulator,
            std::move(wrench_forecast)
        );

        if (!adaptor) {
            std::cerr << "failed to create raisim dynamics actor" << std::endl;
            return nullptr;
        }
    }
    else if (configuration.type == Type::PINOCCHIO) {
        if (!configuration.pinocchio) {
            std::cerr << "selected pinocchio actor without configuration" << std::endl;
            return nullptr;
        }

        adaptor = PinocchioAdaptor::create(
            *configuration.pinocchio,
            simulator,
            std::move(wrench_forecast)
        );

        if (!adaptor) {
            std::cerr << "failed to create pinocchio dynamics actor" << std::endl;
            return nullptr;
        }
    }
    else {
        std::cerr << "unknown actor dynamics type " << configuration.type << std::endl;
        return nullptr;
    }

    return adaptor;
}

} // namespace FrankaRidgeback
