#include "controller/filterqp.hpp"


FilterQP::FilterQP() 
        : m_joint_velocities(10)
        , m_joint_limits_slack_variables(7)
        , m_self_collision_slack_variables(7)
        , m_arm_reach_slack_variables(1)
        , m_constraints(6)
    {
        m_optimisation_variables = m_joint_velocities +
                                 m_joint_limits_slack_variables +
                                 m_self_collision_slack_variables +
                                 m_arm_reach_slack_variables;

        m_qp = QuadraticProgram::create(m_optimisation_variables, m_constraints);
    }

void FilterQP::filter(       
        Eigen::Ref<Eigen::VectorXd> control, 
        const Eigen::Ref<Eigen::VectorXd> state, 
        const Vector6d external_wrench, 
        double time
    )
    {

        
    }

    