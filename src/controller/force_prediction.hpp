#pragma once

#include <deque>

#include "controller/kalman.hpp"

/**
 * @brief A predictor that observes forces and estimates future forces.
 */
class ForcePredictor
{
    /**
     * @brief Update the predictor with an observed force.
     * 
     * @pre `time > previous_update_time`
     * 
     * @param force The observed force.
     * @param time The time of the observation.
     */
    virtual void update(Eigen::Vector3d force, double time) = 0;

    /**
     * @brief Update the predictor without an observed force.
     * 
     * @pre `time > previous_update_time`
     * 
     * @param time The time of the update.
     */
    virtual void update(double time) = 0;

    /**
     * @brief Predict the force at a time in the future.
     * 
     * @param time The time of the force prdiction
     * @returns The predicted force.
     */
    virtual Eigen::Vector3d predict(double time) = 0;
};

/**
 * @brief A trivial constant force, that returns the observed force.
 */
class ConstantForcePredictor : public ForcePredictor
{
public:

    /**
     * @brief Update the predictor with an observed force.
     * 
     * @param force The observed force.
     * @param time The time of the observation.
     */
    inline void update(Eigen::Vector3d force, double time) override {
        m_force = force;
    }

    /**
     * @brief Does nothing.
     */
    inline void update(double time) override {}

    /**
     * @brief Predict the force as the last observed force, regardless of
     * time.
     * 
     * @param time The time of the force prdiction
     * @returns The last observed force.
     */
    inline Eigen::Vector3d predict(double /* time */) override {
        return m_force;
    }

private:

    /// The last observed force.
    Eigen::Vector3d m_force;
};

// class AverageForcePredictor : public ForcePredictor
// {
// public:

// private:

//     std::deque<Eigen::Vector3d> m_forces;
// };

/**
 * @brief A force predictor based on a kalman filter.
 * 
 * Note that update() must be called every configured time_step, with or without
 * a force observation.
 */
class KalmanForcePredictor : public ForcePredictor
{
public:

    struct Configuration {

        /// The time step of the state transition matrix for the force.
        double time_step;

        /// The horison over which the force should be predicted.
        double horison;

        /// The order of the force estimation.
        unsigned int order;

        /// Set to {var(x), var(y), var(z), var(dx), var(dy), var(dz), ...}
        Eigen::VectorXd transition_variance;

        /// Set to {var(x), var(y), var(z), var(dx), var(dy), var(dz), ...}
        Eigen::VectorXd observation_variance;
    };

    /**
     * @brief Create a new kalman force predictor.
     * 
     * @param configuration The configuration of the predictor.
     * @returns A pointer to the predictor on success or nullptr on failure.
     */
    std::unique_ptr<KalmanForcePredictor> create(
        const Configuration &configuration
    );

    /**
     * @brief Create a state transition matrix of a given order.
     * 
     * The order is the highest derivative in each observation, that is used to
     * predict the force.
     * 
     * @param time_step The time step of the transition matrix.
     * @param order The order of the state transition matrix.
     * 
     * @returns The state transition matrix.
     */
    Eigen::MatrixXd create_euler_state_transition_matrix(
        double time_step,
        unsigned int order
    );

    /**
     * @brief Update the predictor with an observed force.
     * 
     * @param force The observed force.
     * @param time The time of the observation.
     */
    inline void update(Eigen::Vector3d force, double time) override;

    /**
     * @brief Update the predictor without a force.
     */
    inline void update(double time) override;

    /**
     * @brief Predict the force as the last observed force, regardless of
     * time.
     * 
     * @param time The time of the force prdiction
     * @returns The last observed force.
     */
    Eigen::Vector3d predict(double /* time */) override;

private:

    KalmanForcePredictor() = default;

    /// The period of time over which the prediction is made.
    double m_horison;

    /// The expected kalman time step.
    double m_time_step;

    /// The number of steps to make in each update.
    unsigned int m_steps;

    /// The time of the last update.
    double m_last_update;

    /// The kalman filter used to estimate the current force.
    std::unique_ptr<KalmanFilter> m_kalman;

    /// The kalman filter used to estimate future forces.
    std::unique_ptr<KalmanFilter> m_predictor;

    /// The calculated state, storing force derivatives.
    Eigen::VectorXd m_state;

    /// The latest updated horison.
    Eigen::MatrixXd m_prediction;
};
