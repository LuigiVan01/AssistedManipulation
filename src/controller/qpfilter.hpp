#pragma once

#include <cmath>


class QP_Filter
{
public:
    void filter(
        Eigen::Ref<Eigen::VectorXd> control, 
        const Eigen::Ref<Eigen::VectorXd> state, 
        const Vector6d external_wrench, 
        double time
    )
    {
        

    }
};