#include "controller/kalman.hpp"

#include <iostream>

std::unique_ptr<KalmanFilter> KalmanFilter::create(
    const KalmanFilter::Configuration &configuration
) {
    static auto check_dimensions = [](
        const char *name,
        const Eigen::MatrixXd &matrix,
        std::int64_t rows,
        std::int64_t cols
    ){
        bool is_valid = matrix.rows() == rows && matrix.cols() == cols;

        if (!is_valid) {
            std::cerr
                << "invalid " << name << " dimensions ("
                << matrix.rows() << ", " << matrix.cols()
                << ") expected (" << rows << ", " << cols << ")"
                << std::endl;
        }

        return is_valid;
    };

    bool valid = check_dimensions(
        "state transition matrix",
        configuration.state_transition_matrix,
        configuration.states,
        configuration.states
    );

    valid &= check_dimensions(
        "control transition matrix",
        configuration.control_transition_matrix,
        configuration.states,
        configuration.controls
    );

    valid &= check_dimensions(
        "transition covariance matrix",
        configuration.transition_covariance,
        configuration.states,
        configuration.states
    );

    valid &= check_dimensions(
        "observation matrix",
        configuration.observation_matrix,
        configuration.observed_states,
        configuration.states
    );

    valid &= check_dimensions(
        "observation covariance matrix",
        configuration.observation_covariance,
        configuration.observed_states,
        configuration.states
    );

    if (!valid)
        return nullptr;

    return std::unique_ptr<KalmanFilter>(new KalmanFilter(configuration));
}

void KalmanFilter::update(
    Eigen::Ref<Eigen::VectorXd> observation,
    Eigen::Ref<Eigen::VectorXd> control
) {
    assert(observation.size() == m_observed_state_size);
    assert(control.size() == m_control_size);

    // Calculate the optimal kalman gain.
    auto optimal_kalman_gain = (
        m_covariance * m_observation_matrix.transpose() * (
            m_observation_matrix * m_covariance * m_observation_matrix.transpose() +
            m_observation_covariance
        ).inverse()
    );

    // Correct the previously predicted state estimation, by interpolating
    // between the estimated state and the observed state.
    m_state = m_next_state + optimal_kalman_gain * (
        m_observation_matrix * observation - m_observation_matrix * m_next_state
    );

    // Update the noise covariance of the estimated state. Simplified update
    // when the kalman gain is optimal. See
    // https://en.wikipedia.org/wiki/Kalman_filter#Derivations
    m_covariance = (
        m_identity - optimal_kalman_gain * m_observation_matrix
    ) * m_covariance;

    // Predict the next state from the current state and control.
    m_next_state = (
        m_state_transition_matrix * m_state +
        m_control_transition_matrix * control
    );

    // Extrapolate the noise to the next state.
    m_covariance = (
        m_state_transition_matrix * m_covariance * m_state_transition_matrix.transpose() + 
        m_transition_covariance
    );
}