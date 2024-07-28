#pragma once

#include <mutex>
#include <stdexcept>

#include <Eigen/Eigen>

namespace controller {

/**
 * @brief A PID controller.
 */
class PID
{
public:

    struct Configuration {

        /// Proportional gain matrix.
        Eigen::VectorXd kp;

        /// Derivative gain matrix.
        Eigen::VectorXd kd;

        /// Integral gain matrix.
        Eigen::VectorXd ki;

        /// The minimum control output.
        Eigen::VectorXd minimum;

        /// The maximum control output.
        Eigen::VectorXd maximum;
    };

    /**
     * @brief Create a new PID controller.
     * 
     * @param reference The desired target of the controller.
     * 
     * @throws std::logic_error if the pid parameters are not square or have
     * different dimensions.
     */
    inline PID(
        const Configuration &configuration,
        Eigen::VectorXd reference,
        double time
      ) : m_kp(configuration.kp)
        , m_kd(configuration.kd)
        , m_ki(configuration.ki)
        , m_minimum(configuration.minimum)
        , m_maximum(configuration.maximum)
        , m_reference(reference)
        , m_error(Eigen::VectorXd::Zero(configuration.kp.size())) // Vector of zeros with correct dimension.
        , m_last_error(Eigen::VectorXd::Zero(configuration.kp.size()))
        , m_cumulative_error(Eigen::VectorXd::Zero(configuration.kp.size()))
        , m_last_time(time)
    {
        bool equal_dimensions = (
            m_kp.size() == m_kd.size() &&
            m_kd.size() == m_ki.size() &&
            m_ki.size() == m_minimum.size() &&
            m_minimum.size() == m_maximum.size()
        );

        if (!equal_dimensions) {
            throw std::logic_error(
                "pid parameters must be square and have the same dimensions"
            );
        }
    }

    /**
     * @brief Updated the desired reference state of the pid controller.
     * @param state The desired reference state.
     */
    inline void set_reference(Eigen::Ref<Eigen::VectorXd> state) {
        m_reference = state;
    }

    /**
     * @brief Set the controllers proportional gain.
     * @param kp The proportional gain.
     */
    inline void set_proportional_gain(Eigen::Ref<Eigen::VectorXd> kp) {
        m_kp = kp;
    }

    /**
     * @brief Set the controllers derivative gain.
     * @param kd The derivative gain.
     */
    inline void set_derivative_gain(Eigen::Ref<Eigen::VectorXd> kd) {
        m_kd = kd;
    }

    /**
     * @brief Set the controllers integral gain.
     * @param ki The integral gain.
     */
    inline void set_integral_gain(Eigen::Ref<Eigen::VectorXd> ki) {
        m_ki = ki;
    }

    /**
     * @brief Update the control to apply to the current observed state.
     * 
     * Both the state and control must have the same dimensions as the pid
     * parameters.
     * 
     * @param state The observed state of the system.
     * @param control The output control to be set by this function.
     * @param time The current time in seconds.
     */
    inline void update(
        Eigen::Ref<Eigen::VectorXd> state,
        Eigen::Ref<Eigen::VectorXd> control,
        double time
    ) {
        double dt = time - m_last_time;
        m_error = m_reference - state;

        // Runge kutta? Inaccuracy of small number division?

        // Calculate control and saturate.
        control = (
            m_kp * m_error +
            m_kd * (m_error - m_last_error) / dt +
            m_ki * m_cumulative_error
        ).cwiseMin(m_maximum).cwiseMax(m_minimum);

        // Calculate saturation per degree of freedom.
        Eigen::MatrixXd not_saturated = (
            control.array() < m_maximum.array() &&
            control.array() > m_minimum.array()
        ).cast<double>();

        // Accumulate error only if not saturated (anti windup).
        m_cumulative_error += m_error * dt * not_saturated;

        m_last_error = m_error;
        m_last_time = time;
    }

private:

    /// Proportional gain.
    Eigen::VectorXd m_kp;

    /// Derivative gain.
    Eigen::VectorXd m_kd;

    /// Integral gain.
    Eigen::VectorXd m_ki;

    /// Minimum control output.
    Eigen::VectorXd m_minimum;

    /// Maximum control output.
    Eigen::VectorXd m_maximum;

    /// The reference state.
    Eigen::VectorXd m_reference;

    /// Cached storage for the error signal.
    Eigen::VectorXd m_error;

    /// The previously calculated error, for derivative calculation.
    Eigen::VectorXd m_last_error;

    /// The cumulative error, for integral calculation.
    Eigen::VectorXd m_cumulative_error;

    /// The time of the last update.
    double m_last_time;
};

}
