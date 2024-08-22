#pragma once

#include <cstdint>

#include <Eigen/Eigen>

/**
 * @brief A Kalman filter.
 * 
 * Various resources:
 * - https://www.kalmanfilter.net/multiSummary.html
 * - https://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf
 */
class KalmanFilter
{
public:

    struct Configuration {

        /// The number of measured states.        
        unsigned int observed_states;

        /// The number of estimated states.
        unsigned int states;

        /// The number of control inputs.
        unsigned int controls;

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

    /**
     * @brief Create a new kalman filter.
     * 
     * Validates the Kalman matrix dimensions.
     * 
     * @param configuration The various matricies used in the filter.
     * @return A pointer to the filter on success or nullptr on failure.
     */
    std::unique_ptr<KalmanFilter> create(const Configuration &configuration);

    /**
     * @brief Get the size of the observed state vector.
     */
    inline unsigned int get_observed_state_size() {
        return m_observed_state_size;
    }

    /**
     * @brief Get the size of the control vector.
     */
    inline unsigned int get_control_size() {
        return m_control_size;
    }

    /**
     * @brief Get the state size object
     */
    inline unsigned int get_estimated_state_size() {
        return m_estimated_state_size;
    }

    /**
     * @brief Get the latest state estimation.
     */
    const Eigen::VectorXd &get_estimation() {
        return m_state;
    }

    /**
     * @brief Get the latest estimation covariance noise.
     */
    const Eigen::MatrixXd &get_covariance() {
        return m_covariance;
    }

    /**
     * @brief Update the Kalman filter with an observation.
     * 
     * @param observation The observed state.
     * @param control The control parameters.
     */
    inline void update(
        Eigen::Ref<Eigen::VectorXd> observation,
        Eigen::Ref<Eigen::VectorXd> control
    );

private:

    KalmanFilter(const Configuration &config)
       : m_observed_state_size(config.observed_states)
       , m_control_size(config.controls)
       , m_estimated_state_size(config.states)
       , m_state_transition_matrix(config.state_transition_matrix)
       , m_control_transition_matrix(config.control_transition_matrix)
       , m_transition_covariance(config.transition_covariance)
       , m_observation_matrix(config.observation_matrix)
       , m_observation_covariance(config.observation_covariance)
       , m_identity(Eigen::MatrixXd::Identity(config.states, config.states))
       , m_covariance()
       , m_state(config.initial_state)
    {}

    /// The number of states in each observation.
    const unsigned int m_observed_state_size;

    /// The number of control parameters.
    const unsigned int m_control_size;

    /// The number of estimated states.
    const unsigned int m_estimated_state_size;

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

    /// Identity matrix used in transition noise covariance matrix.
    const Eigen::MatrixXd m_identity;

    /// Noise of state estimation.
    Eigen::MatrixXd m_covariance;

    /// The most recently estimated state.
    Eigen::VectorXd m_state;

    /// The most recently estimated next state.
    Eigen::VectorXd m_next_state;
};
