#pragma once

#include "controller/json.hpp"
#include "controller/mppi.hpp"
#include "frankaridgeback/dynamics.hpp"

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

        // JSON conversion for track point objective configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, point)
    };

    inline TrackPoint(const Eigen::Vector3d point)
        : m_point(point)
    {}

    TrackPoint(const TrackPoint &) = default;

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
    static inline std::unique_ptr<TrackPoint> create(
        const Configuration &configuration
    ) {
        return std::make_unique<TrackPoint>(configuration.point);
    }

    /**
     * @brief Get the cost of a state and control input over dt.
     * 
     * @param state The state of the system.
     * @param control The control parameters applied to the state.
     * @param time The current time.
     * 
     * @returns The cost of the step.
     */
    double get_cost(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        mppi::Dynamics *dynamics,
        double time
    ) override;

    void reset() override {};

    inline std::unique_ptr<mppi::Cost> copy() override {
        return std::make_unique<TrackPoint>(*this);
    }

private:

    /// The point in space to track.
    Eigen::Vector3d m_point;
};
