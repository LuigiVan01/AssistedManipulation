#pragma once

#include <cmath>

#include "controller/eigen.hpp"
#include "osqp.h"
#include "controller/qp.hpp"


class FilterQP: public QuadraticProgram
{
public:

    FilterQP();



    void filter(
        Eigen::Ref<Eigen::VectorXd> control, 
        const Eigen::Ref<Eigen::VectorXd> state, 
        const Vector6d external_wrench, 
        double time
    );

    
 

    void create();

    void set_control_ref(Eigen::Ref<Eigen::VectorXd> control_ref);

    void update_qp(
        const Eigen::Ref<Eigen::VectorXd> state, 
        const Vector6d external_wrench, 
        double time
    );

private:

    //
    int m_joint_velocities;

    int m_joint_limits_slack_variables;
    int m_self_collision_slack_variables; 
    int m_arm_reach_slack_variables;
    
    // Number of variables to optimise for
    int m_optimisation_variables;
    // Number of constraints
    int m_constraints;

    std::unique_ptr<QuadraticProgram> m_qp;

    std::vector<int> m_joint_velocities_weight = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
    std::vector<int> m_joint_limits_slack_variables_weight = {2, 2, 2, 2, 2, 2, 2};
    std::vector<int> m_self_collision_slack_variables_weight = {2, 2, 2, 2, 2, 2, 2};
    std::vector<int> m_arm_reach_slack_variables_weight = {2, 2, 2, 2, 2, 2, 2};


    // 
    Eigen::VectorXd m_control_ref;

};