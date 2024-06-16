#pragma once

#include "mppi.hpp"
#include "dynamics.hpp"

/**
 * @brief Cost function of the panda research 3 ridgeback assisted manipulation
 * task.
 */
class Cost : public mppi::Cost<FrankaRidgeback::DoF::STATE, FrankaRidgeback::DoF::CONTROL>
{
public:

    /**
     * @brief Create an instance of the assisted object manipulation cost
     * function.
     * 
     * @param urdf The robot definition file to instantiate the robot kinematics
     * from, to calculate the cost from.
     * 
     * @return A pointer to the cost instance on success, or nullptr on failure.
     */
    static std::shared_ptr<Cost> create(const std::string &urdf);

    /**
     * @brief Update the cost given a subsequent state from a control input.
     * 
     * @param state The next state of the system.
     * @param control The controls used to achieve the state.
     * @param t The change in time.
     * 
     * @returns The cost of the step.
     */
    double step(const State &state, const Control &control, double dt) override;

    /**
     * @brief Reset the cost.
     */
    void reset() override;

    /**
     * @brief Get the total cumulative cost.
     * @return The cumulative cost.
     */
    double cost() override;

private:

    /**
     * @brief Initialise the assisted manipulation cost.
     * @param model Pointer to the application kinematics / dynamics model.
     */
    Cost(std::unique_ptr<FrankaRidgeback::Model> &&model);

    /// Pointer to the model to derive cost from.
    std::unique_ptr<FrankaRidgeback::Model> m_model;

    /// The point in space for the end effector to reach.
    Eigen::Vector3d m_point;
};