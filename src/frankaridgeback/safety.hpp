#pragma once

#include <Eigen/Eigen>

#include "frankaridgeback/dof.hpp"

class FrankaRidgebackSafetyFilter
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

    Eigen::Vector<double, FrankaRidgeback::DoF::CONTROL> filter(
        Eigen::Ref<Eigen::VectorXd> state,
        Eigen::Ref<Eigen::VectorXd> control,
        double time
    );

private:

};
