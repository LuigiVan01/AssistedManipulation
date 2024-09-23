#pragma once

#include <mutex>
#include <stdexcept>

#include <Eigen/Eigen>

#include "controller/json.hpp"

namespace controller {

/**
 * @brief A PID controller.
 */
class PID
{
public:

    struct Configuration {

        /// The number of degrees of freedom for state.
        unsigned int state_dof;

        /// The number of degrees of freedom for control.
        unsigned int control_dof;

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

        /// The initial reference of the pid controller.
        Eigen::VectorXd reference;

        /// The initial time.
        double time;

        // JSON conversion for pid configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            state_dof, control_dof, kp, kd, ki, minimum, maximum, reference,
            time
        )
    };

    /**
     * @brief Create a new PID controller.
     * 
     * @param reference The desired target of the controller.
     * 
     * @throws std::logic_error if the pid parameters are not square or have
     * different dimensions.
     */
    inline PID(const Configuration &configuration)
        : m_kp(configuration.kp)
        , m_kd(configuration.kd)
        , m_ki(configuration.ki)
        , m_minimum(configuration.minimum)
        , m_maximum(configuration.maximum)
        , m_reference(configuration.reference)
        , m_error(Eigen::VectorXd::Zero(configuration.state_dof))
        , m_last_error(Eigen::VectorXd::Zero(configuration.state_dof))
        , m_cumulative_error(Eigen::VectorXd::Zero(configuration.state_dof))
        , m_saturation(Eigen::VectorXd::Zero(configuration.control_dof))
        , m_control(Eigen::VectorXd::Zero(configuration.control_dof))
        , m_last_time(configuration.time)
    {
        bool equal_dimensions = (
            m_kp.size() == m_kd.size() &&
            m_kd.size() == m_ki.size() &&
            m_ki.size() == m_minimum.size() &&
            m_minimum.size() == m_maximum.size() &&
            m_reference.size() == m_minimum.size()
        );

        if (!equal_dimensions) {
            throw std::logic_error(
                "pid parameters must be square and have the same dimensions"
            );
        }
    }

    /**
     * @brief Update the control to apply to the current observed state.
     * 
     * Both the state and control must have the same dimensions as the pid
     * parameters.
     * 
     * @param state The observed state of the system.
     * @param time The current time in seconds.
     */
    inline void update(Eigen::Ref<Eigen::VectorXd> state, double time)
    {
        // Time must be monotonically increasing. Also waits until dt
        // calculations are valid.
        if (time <= m_last_time)
            return;

        double dt = time - m_last_time;
        auto error = m_reference - state;

        // Runge kutta? Inaccuracy of small number division?

        // Calculate control and saturate.
        m_control = (
            m_kp.cwiseProduct(error) +
            m_kd.cwiseProduct(error - m_last_error) / dt +
            m_ki.cwiseProduct(m_cumulative_error)
        ).cwiseMin(m_maximum).cwiseMax(m_minimum);

        // Calculate saturation per degree of freedom.
        m_saturation = (
            m_control.array() < m_maximum.array() &&
            m_control.array() > m_minimum.array()
        ).cast<double>();

        // Accumulate error only if not saturated (anti windup).
        m_cumulative_error += error.cwiseProduct(m_saturation) * dt;

        m_last_error = error;
        m_last_time = time;
    }

    inline const Eigen::VectorXd get_control() const {
        return m_control;
    }

    inline double get_time() const {
        return m_last_time;
    }

    inline const Eigen::VectorXd &get_reference() const {
        return m_reference;
    }

    inline const Eigen::VectorXd &get_error() const {
        return m_last_error;
    }

    inline const Eigen::VectorXd &get_cumulative_error() const {
        return m_cumulative_error;
    }

    inline const Eigen::VectorXd &get_saturation() const {
        return m_saturation;
    }

    inline const Eigen::VectorXd &get_proportional_gain() const {
        return m_kp;
    }

    inline const Eigen::VectorXd &get_derivative_gain() const {
        return m_kd;
    }

    inline const Eigen::VectorXd &get_integral_gain() const {
        return m_ki;
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

    /// The control action of the last update.
    Eigen::VectorXd m_saturation;

    /// The control action of the last update.
    Eigen::VectorXd m_control;

    /// The time of the last update.
    double m_last_time;
};

} // namespace controller
