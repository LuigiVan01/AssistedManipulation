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

    private:



        int m_joint_velocities;

        int m_joint_limits_slack_variables;
        int m_self_collision_slack_variables; 
        int m_arm_reach_slack_variables;

        int m_optimisation_variables;
        int m_constraints;

        std::unique_ptr<QuadraticProgram> m_qp;

    
    


};