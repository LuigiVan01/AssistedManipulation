#pragma once

#include "simulation/frankaridgeback/dynamics.hpp"

namespace FrankaRidgeback {

/**
 * @brief An interface used by the frankaridgeback actor to use to simulate the
 * true model.
 * 
 * Enables the usage of non-raisim simulation of the actor dynamics.
 */
class ActorDynamics
{
public:

    virtual ~ActorDynamics() = default;

protected:

    // Only the FrankaRidgeback::Actor class can create actor dynamics instances.
    friend class Actor;

    inline ActorDynamics(Simulator *simulator)
        : m_simulator(simulator)
    {}

    /**
     * @brief Create a new simulator adaptor.
     * 
     * @param configuration The configuration of the adaptor.
     * @param simulator The simulator to which the adaptor belongs.
     */
    static std::unique_ptr<ActorDynamics> create(
        const SimulatorDynamics::Configuration &configuration,
        Simulator *simulator
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
class RaisimActorDynamics : public ActorDynamics
{
public:

    static inline const std::vector<std::pair<Link, double>>
    VISUAL_COLLISION_LINKS {{
        {Link::PANDA_LINK1, 0.1},
        {Link::PANDA_LINK2, 0.1},
        {Link::PANDA_LINK3, 0.1},
        {Link::PANDA_LINK4, 0.1},
        {Link::PANDA_LINK5, 0.1},
        {Link::PANDA_LINK6, 0.1},
        {Link::PANDA_LINK7, 0.1},
        {Link::PIVOT, 0.75}
    }};

    inline static std::unique_ptr<RaisimActorDynamics> create(
        RaisimDynamics::Configuration configuration,
        Simulator *simulator
    );

    /**
     * @brief Get a pointer to the underlying dynamics.
     */
    inline Dynamics *get_dynamics() override
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
        // for (int i = 0; i < m_collision_spheres.size(); i++) {
        //     m_collision_spheres[i]->setPosition(
        //         m_dynamics->get_link_position(VISUAL_COLLISION_LINKS[i].first)
        //     );
        // }
    }

private:

    inline RaisimActorDynamics(
        Simulator *simulator,
        std::unique_ptr<RaisimDynamics> &&dynamics
      ) : ActorDynamics(simulator)
        , m_dynamics(std::move(dynamics))
    {
        // for (auto [link, radius] : VISUAL_COLLISION_LINKS) {
        //     auto sphere = simulator->get_server().addVisualSphere(
        //         LINK_NAMES[(std::size_t)link] + "_collsion_sphere",
        //         radius, 1.0, 0.0, 0.0, 0.3
        //     );
        //     sphere->setPosition(m_dynamics->get_link_position(link));
        //     m_collision_spheres.push_back(sphere);
        // }
    }

    std::vector<raisim::Visuals*> m_collision_spheres;

    std::unique_ptr<RaisimDynamics> m_dynamics;
};

/**
 * @brief Provides pinocchio dynamics visualisation with raisim.
 * 
 * @todo Refactor so not to copy all the derived functions.
 */
class PinocchioActorDynamics : public ActorDynamics
{
public:

    /**
     * @brief Create a new raisim pinocchio dynamics adaptor.
     * 
     * @param configuration The configuration of the pinocchio dynamics.
     * @param simulator Pointer to the owning simulator.
     */
    inline static std::unique_ptr<PinocchioActorDynamics> create(
        const PinocchioDynamics::Configuration &configuration,
        Simulator *simulator
    );

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

    inline ~PinocchioActorDynamics()
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
    inline PinocchioActorDynamics(
        Simulator *simulator,
        std::unique_ptr<PinocchioDynamics> &&pinocchio,
        raisim::ArticulatedSystemVisual *visual
      ) : ActorDynamics(simulator)
        , m_dynamics(std::move(pinocchio))
        , m_visual(visual)
    {}

    /// The pinocchio dynamics.
    std::unique_ptr<PinocchioDynamics> m_dynamics;

    /// Visualisation of the pinocchio dynamics.
    raisim::ArticulatedSystemVisual *m_visual;
};

} // namespace FrankaRidgeback
