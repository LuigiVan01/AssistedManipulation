#pragma once

#include <Eigen/Eigen>

class Kalman
{
public:

    struct Configuration {

        Eigen::MatrixXd state_transition_model;

        Eigen::MatrixXd observation_model;

        Eigen::MatrixXd process_noise_covariance;

        Eigen::MatrixXd observation_noise_covariance;
    };

private:

    void update(Eigen::Ref<Eigen::VectorXd> observation);
};
