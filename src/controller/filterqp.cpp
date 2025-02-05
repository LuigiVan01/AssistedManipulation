#include "controller/filterqp.hpp"


FilterQP::FilterQP() 
        : m_joint_velocities(10)
        , m_joint_limits_slack_variables(7)
        , m_self_collision_slack_variables(7)
        , m_arm_reach_slack_variables(1)
        , m_optimisation_variables (m_joint_velocities +
                                 m_joint_limits_slack_variables +
                                 m_self_collision_slack_variables +
                                 m_arm_reach_slack_variables)
        , m_constraints(6)
        , m_qp (QuadraticProgram::create(m_optimisation_variables, 
                                                   m_constraints))
    {

        


        /// Hesian matrix filling
        int current_pos = 0;

        // Fill joint velocities weights
        for(int i = 0; i < m_joint_velocities; i++) {
            m_qp->hessian.diagonal()(current_pos + i) 
            = m_joint_velocities_weight[i];
        }
        current_pos += m_joint_velocities;

        // Fill joint limits slack weights
        for(int i = 0; i < m_joint_limits_slack_variables; i++) {
            m_qp->hessian.diagonal()(current_pos + i) 
            = m_joint_limits_slack_variables_weight[i];
        }
        current_pos += m_joint_limits_slack_variables;

        // Fill self collision slack weights
        for(int i = 0; i < m_self_collision_slack_variables; i++) {
            m_qp->hessian.diagonal()(current_pos + i) 
            = m_self_collision_slack_variables_weight[i];
        }
        current_pos += m_self_collision_slack_variables;

        // Fill arm reach slack weights
        for(int i = 0; i < m_arm_reach_slack_variables; i++) {
            m_qp->hessian.diagonal()(current_pos + i) 
            = m_arm_reach_slack_variables_weight[i];
        }

    }

void FilterQP::set_control_ref(Eigen::Ref<Eigen::VectorXd> control_ref)
{
    m_control_ref = control_ref;

    
}

void FilterQP::update_qp(
        const Eigen::Ref<Eigen::VectorXd> state, 
        const Vector6d external_wrench, 
        double time
    )
    {
        // The hessian does not change with the mppi reference trajectory

        // Linear vector filling
        for(int i = 0; i < m_joint_velocities; i++) {
            m_qp->linear(i) = -m_joint_velocities_weight[i]*m_control_ref[i];
        }

    }

void FilterQP::filter(       
        Eigen::Ref<Eigen::VectorXd> control, 
        const Eigen::Ref<Eigen::VectorXd> state, 
        const Vector6d external_wrench, 
        double time
    )
    {
        // Set the mppi trajecotry as the reference for the QP
        set_control_ref(control);

        // Update the QP with the new reference
        update_qp(state, external_wrench, time);
        
    }

    