#pragma once

#include <memory>
#include <cstdint>

#include "controller/eigen.hpp"

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

        /// Maps a state observation to the next state.
        MatrixXd state_transition_matrix;

        /// The noise of the state transition mapping.
        MatrixXd transition_covariance;

        /// Maps a state to an observation.
        MatrixXd observation_matrix;

        /// The noise of the observation mapping.
        MatrixXd observation_covariance;

        /// The initial state stored in the filter.
        VectorXd initial_state;

        // The initial state covariance.
        MatrixXd initial_covariance;
    };

    /**
     * @brief Create a new kalman filter.
     * 
     * Validates the Kalman matrix dimensions.
     * 
     * @param configuration The various matricies used in the filter.
     * @return A pointer to the filter on success or nullptr on failure.
     */
    static std::unique_ptr<KalmanFilter> create(const Configuration &configuration);

    /**
     * @brief Get the size of the observed state vector.
     */
    inline unsigned int get_observed_state_size() {
        return m_observed_state_size;
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
    const VectorXd &get_estimation() {
        return m_state;
    }

    /**
     * @brief Get the latest estimation covariance noise.
     */
    const MatrixXd &get_covariance() {
        return m_covariance;
    }

    /**
     * @brief Set the estimated state in the kalman filter.
     * @param state The estimated state to set to.
     */
    inline void set_estimation(const VectorXd &state) {
        m_state = state;
        m_next_state = m_state_transition_matrix * state;
    }

    /**
     * @brief Set the covariance of the kalman filter.
     */
    inline void set_covariance(const MatrixXd &covariance) {
        m_covariance = covariance;
    }

    /**
     * @brief Update the filter with an observation.
     * 
     * The observation is not a state of the system! The observation matrix maps
     * the state to an observation. The observation passed should already be an
     * observation, not a state.
     * 
     * Updates the state and error based on the previous state, previous state
     * confidence and an observation.
     * 
     * @param observation The observed state.
     */
    void update(VectorXd &observation);

    /**
     * @brief Predict the subsequent state.
     * 
     * Note that the covariance of the state increases after each prediction
     * and predictions will become increasingly inaccurate.
     * 
     * Performs an update with the kalman gain to zero to rely only on the
     * process model / state transition matrix.
     * 
     * @param update_covariance Whether to update the covariance matrix.
     */
    void predict(bool update_covariance = true);

private:

    KalmanFilter(const Configuration &config);

    /// The number of states in each observation.
    const unsigned int m_observed_state_size;

    /// The number of estimated states.
    const unsigned int m_estimated_state_size;

    /// Maps a state observation to the next state.
    const MatrixXd m_state_transition_matrix;

    /// The noise of the state transition mapping.
    const MatrixXd m_transition_covariance;

    /// Maps a state to an observation.
    const MatrixXd m_observation_matrix;

    /// The noise of the observation mapping.
    const MatrixXd m_observation_covariance;

    /// Identity matrix used in transition noise covariance matrix.
    const MatrixXd m_identity;

    /// Noise of state estimation.
    MatrixXd m_covariance;

    /// The most recently estimated state.
    VectorXd m_state;

    /// The most recently estimated next state.
    VectorXd m_next_state;
};
