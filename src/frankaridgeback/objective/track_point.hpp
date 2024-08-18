#pragma once

#include "controller/json.hpp"
#include "controller/mppi.hpp"
#include "frankaridgeback/model.hpp"

/**
 * @brief Cost function of the panda research 3 ridgeback assisted manipulation
 * task.
 */
class TrackPoint : public mppi::Cost
{
public:

    struct Configuration {

        /// The position in 3D space to track.
        Eigen::Vector3d point;

        /// The configuration of the model.
        FrankaRidgeback::Model::Configuration model;

        // JSON conversion for track point objective configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, point, model)
    };

    inline constexpr int state_dof() override {
        return FrankaRidgeback::DoF::STATE;
    }

    inline constexpr int control_dof() override {
        return FrankaRidgeback::DoF::CONTROL;
    }

    /**
     * @brief Create a track point objective function.
     * 
     * @param configuration The configuration of the objective function.
     * @return A pointer to the cost instance on success, or nullptr on failure.
     */
    static std::unique_ptr<TrackPoint> create(const Configuration &configuration);

    /**
     * @brief Get the cost of a state and control input over dt.
     * 
     * @param state The state of the system.
     * @param control The control parameters applied to the state.
     * @param time The current time.
     * 
     * @returns The cost of the step.
     */
    double get(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        double time
    ) override;

    void reset() override {};

    std::unique_ptr<FrankaRidgeback::Model> &model() {
        return m_model;
    }

    inline std::unique_ptr<mppi::Cost> copy() override {
        return std::unique_ptr<TrackPoint>(
            new TrackPoint(m_point, std::move(m_model->copy()))
        );
    }

private:

    /**
     * @brief Initialise the assisted manipulation cost.
     * 
     * @param point The point in world space to track.
     * @param model Pointer robot model.
     */
    TrackPoint(
        Eigen::Vector3d point,
        std::unique_ptr<FrankaRidgeback::Model> &&model
    );

    /// The point in space to track.
    Eigen::Vector3d m_point;

    /// Pointer to the model to calculate proximity to the point.
    std::unique_ptr<FrankaRidgeback::Model> m_model;
};
