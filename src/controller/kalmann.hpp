#pragma once

#include <Eigen/Eigen>

class KalmanFilter
{
public:

    struct Configuration {

        /// Maps a state observation to the next state.
        Eigen::MatrixXd state_transition_matrix;

        /// Maps a control input to the next state.
        Eigen::MatrixXd control_transition_matrix;

        /// The noise of the state transition mapping.
        Eigen::MatrixXd transition_covariance;

        /// Maps a state to an observation.
        Eigen::MatrixXd observation_matrix;

        /// The noise of the observation mapping.
        Eigen::MatrixXd observation_covariance;

        /// The initial state stored in the filter.
        Eigen::VectorXd initial_state;
    };

    std::unique_ptr<KalmanFilter> create(const Configuration configuration)
    {
        /// TODO: configuration validation.
        return std::unique_ptr<KalmanFilter>(new KalmanFilter(configuration));
    }

    inline void update(
        Eigen::Ref<Eigen::VectorXd> observation,
        Eigen::Ref<Eigen::VectorXd> control
    ) {
        // Predict the next state:

        // Predict the next state from the current state and control, with
        // additional transition (aka process) noise. Note that the linear
        // combination is allowed due to linear time invariance (LTI).
        // Does add transition noise because it is unknown.
        auto state_estimation = (
            m_state_transition_matrix * m_state +
            m_control_transition_matrix * control
            // + state_transition_noise()
        );

        // Predict the noise of the predicted state.
        auto state_noise_estimation = (
            m_state_transition_matrix * m_covariance * m_state_transition_matrix.transpose() + 
            m_transition_covariance
        );

        // Correct the prediction estimates:

        // 1. Compute the optimal kalman gain.
        auto optimal_kalman_gain = (
            m_covariance * m_observation_matrix.transpose() * (
                m_observation_matrix * m_covariance * m_observation_matrix.transpose() +
                m_observation_covariance
            ).inverse()
        );

        // 2. Update the state as an interpolation between the estimated state and
        // the observed state.
        m_state = m_state + optimal_kalman_gain * (
            m_observation_matrix * observation - m_observation_matrix * m_state
        );

        // 3. Update the uncertainty of the state.

        auto identity = Eigen::MatrixXd::Identity(
            optimal_kalman_gain.rows(),
            optimal_kalman_gain.cols()
        );

        // Used to simplify the following expression, no special meaning.
        auto W = identity - optimal_kalman_gain * m_observation_matrix;

        // Joseph update formula.
        // m_covariance = (
        //     W * state_noise_estimation * W.transpose() +
        //     optimal_kalman_gain * m_observation_covariance * optimal_kalman_gain.tranpose()
        // );

        // Simplified covariance update when the kalman gain is optimal.
        // See https://en.wikipedia.org/wiki/Kalman_filter#Derivations
        m_covariance = W * state_noise_estimation;
    }

private:

    KalmanFilter(const Configuration &config)
       : m_state_transition_matrix(config.state_transition_matrix)
       , m_control_transition_matrix(config.control_transition_matrix)
       , m_transition_covariance(config.transition_covariance)
       , m_observation_matrix(config.observation_matrix)
       , m_observation_covariance(config.observation_covariance)
       , m_covariance()
       , m_state(config.initial_state)
    {}

    /// Maps a state observation to the next state.
    const Eigen::MatrixXd m_state_transition_matrix;

    /// Maps a control input to the next state.
    const Eigen::MatrixXd m_control_transition_matrix;

    /// The noise of the state transition mapping.
    const Eigen::MatrixXd m_transition_covariance;

    /// Maps a state to an observation.
    const Eigen::MatrixXd m_observation_matrix;

    /// The noise of the observation mapping.
    const Eigen::MatrixXd m_observation_covariance;

    /// Noise of state estimation.
    Eigen::MatrixXd m_covariance;

    /// The most recently estimated state.
    Eigen::VectorXd m_state;
};
