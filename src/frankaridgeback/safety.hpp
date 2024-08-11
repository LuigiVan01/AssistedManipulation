#pragma once

#include <Eigen/Eigen>

#include "frankaridgeback/dof.hpp"
#include "controller/mppi.hpp"
#include "controller/qp.hpp"

namespace FrankaRidgeback {

class TrajectorySafetyFilter : public mppi::Filter
{
public:

    struct Configuration {

        Eigen::Vector<double, FrankaRidgeback::DoF::JOINTS> position_minimum;

        Eigen::Vector<double, FrankaRidgeback::DoF::JOINTS> position_maximum;

        Eigen::Vector<double, FrankaRidgeback::DoF::JOINTS> velocity_minimum;

        Eigen::Vector<double, FrankaRidgeback::DoF::JOINTS> velocity_maximum;

        Eigen::Vector<double, FrankaRidgeback::DoF::JOINTS> acceleration_minimum;

        Eigen::Vector<double, FrankaRidgeback::DoF::JOINTS> acceleration_maximum;

        double reach_maximum = 0.8;

        double reach_minimum = 0.15;

        bool limit_joints;

        bool limit_velocity;

        bool limit_acceleration;

        bool limit_reach;
    };

    std::unique_ptr<TrajectorySafetyFilter> create(Configuration &&configuration);

    Eigen::VectorXd filter(
        Eigen::Ref<Eigen::VectorXd> state,
        Eigen::Ref<Eigen::VectorXd> control,
        double time
    ) override;

    /**
     * @brief Reset the filter.
     * 
     * @param state The state of the system.
     * @param time The time.
     */
    void reset(Eigen::Ref<Eigen::VectorXd> state, double time) override;

private:

    TrajectorySafetyFilter() = default;
};

class CriticalSafetyFilter
{
public:

private:

};

} // namespace FrankaRidgeback
