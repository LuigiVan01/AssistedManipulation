#pragma once

#include <mutex>
#include <stdexcept>

#include <Eigen/Eigen>

/**
 * @brief A generic PID controller.
 */
class PID
{
public:

    /**
     * @brief Create a new PID controller.
     * 
     * @param kp Proportional gain matrix.
     * @param kd Derivative gain matrix.
     * @param ki Integral gain matrix.
     * @param reference The desired target of the controller.
     * @param minimum The minimum control output.
     * @param maximum The maximum control output.
     * 
     * @throws std::logic_error if the pid parameters are not square or have
     * different dimensions.
     */
    inline PID(
        Eigen::Ref<Eigen::VectorXd> kp,
        Eigen::Ref<Eigen::VectorXd> kd,
        Eigen::Ref<Eigen::VectorXd> ki,
        Eigen::Ref<Eigen::VectorXd> reference,
        Eigen::Ref<Eigen::VectorXd> minimum,
        Eigen::Ref<Eigen::VectorXd> maximum,
        double time
      ) : m_kp(kp)
        , m_kd(kd)
        , m_ki(ki)
        , m_minimum(minimum)
        , m_maximum(maximum)
        , m_reference(reference) 
        , m_error(kp.Zero()) // A vector of zeros with the correct dimension.
        , m_last_error(kp.Zero())
        , m_cumulative_error(kp.Zero())
        , m_last_time(time)
    {
        bool equal_dimensions = (
            kp.size() == kd.size() &&
            kd.size() == ki.size() &&
            ki.size() == minimum.size() &&
            minimum.size() == maximum.size()
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
        std::scoped_lock lock(m_mutex);
        m_reference = state;
    }

    /**
     * @brief Set the controllers proportional gain.
     * @param kp The proportional gain.
     */
    inline void set_proportional_gain(Eigen::Ref<Eigen::VectorXd> kp) {
        std::scoped_lock lock(m_mutex);
        m_kp = kp;
    }

    /**
     * @brief Set the controllers derivative gain.
     * @param kd The derivative gain.
     */
    inline void set_derivative_gain(Eigen::Ref<Eigen::VectorXd> kd) {
        std::scoped_lock lock(m_mutex);
        m_kd = kd;
    }

    /**
     * @brief Set the controllers integral gain.
     * @param ki The integral gain.
     */
    inline void set_integral_gain(Eigen::Ref<Eigen::VectorXd> ki) {
        std::scoped_lock lock(m_mutex);
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
    )  {
        std::scoped_lock lock(m_mutex);

        m_error = m_reference - state;
        m_cumulative_error += m_error;

        // Runge kutta? Inaccuracy of small number division?
        control = (
            m_kp * m_error +
            m_kd * (m_error - m_last_error) / (time - m_last_time) +
            m_ki * m_cumulative_error
        );

        m_last_error = m_error;
        m_last_time = time;
    }

private:

    /// Mutex protecting concurrent access to the desired state.
    std::mutex m_mutex;

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

    /// The last 
    Eigen::VectorXd m_last_error;

    /// @brief 
    Eigen::VectorXd m_cumulative_error;

    /// The time of the last update.
    double m_last_time;
};
