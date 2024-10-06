#pragma once

#include <mutex>
#include <stdexcept>

#include "controller/eigen.hpp"
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
        unsigned int n;

        /// Proportional gain matrix.
        VectorXd kp;

        /// Derivative gain matrix.
        VectorXd kd;

        /// Integral gain matrix.
        VectorXd ki;

        /// The minimum control output.
        VectorXd minimum;

        /// The maximum control output.
        VectorXd maximum;

        /// The initial reference of the pid controller.
        VectorXd reference;

        /// The initial time.
        double initial_time;

        // JSON conversion for pid configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            n, kp, kd, ki, minimum, maximum, reference, initial_time
        )
    };

    /**
     * @brief An approximation for the characteristics of a human attempting to
     * move an object to a position in space by applying force.
     */
    static inline const Configuration HUMAN_POINT_CONTROL {
        .n = 3,
        .kp = Vector3d(500.0, 500.0, 500.0),
        .kd = Vector3d(50.0, 50.0, 50.0),
        .ki = Vector3d(50.0, 50.0, 50.0),
        .minimum = Eigen::Vector3d(-10000.0, -10000.0, -10000.0),
        .maximum = Eigen::Vector3d(10000.0, 10000.0, 10000.0),
        .reference = Eigen::Vector3d::Zero()
    };

    /**
     * @brief Create a new pid controller.
     * 
     * All the dimensions of the pid controller must be equal.
     * 
     * @param configuration The configuration of the pid controller.
     * @return A pointer to the pid controller on success or nullptr on failure.
     */
    static std::unique_ptr<PID> create(const Configuration &configuration);

    /**
     * @brief Update the control to apply to the current observed state.
     * 
     * @param state The observed state of the system.
     * @param time The current time in seconds.
     */
    virtual void update(const Eigen::Ref<const VectorXd> state, double time);

    inline const VectorXd get_control() const {
        return m_control;
    }

    inline double get_time() const {
        return m_last_time;
    }

    inline const VectorXd &get_reference() const {
        return m_reference;
    }

    inline const VectorXd &get_error() const {
        return m_last_error;
    }

    inline const VectorXd &get_cumulative_error() const {
        return m_cumulative_error;
    }

    inline const VectorXd &get_saturation() const {
        return m_saturation;
    }

    inline const VectorXd &get_proportional_gain() const {
        return m_kp;
    }

    inline const VectorXd &get_derivative_gain() const {
        return m_kd;
    }

    inline const VectorXd &get_integral_gain() const {
        return m_ki;
    }

    /**
     * @brief Updated the desired reference state of the pid controller.
     * @param state The desired reference state.
     */
    inline void set_reference(const Eigen::Ref<const VectorXd> state) {
        m_reference = state;
    }

    /**
     * @brief Set the controllers proportional gain.
     * @param kp The proportional gain.
     */
    inline void set_proportional_gain(const Eigen::Ref<const VectorXd> kp) {
        m_kp = kp;
    }

    /**
     * @brief Set the controllers derivative gain.
     * @param kd The derivative gain.
     */
    inline void set_derivative_gain(const Eigen::Ref<const VectorXd> kd) {
        m_kd = kd;
    }

    /**
     * @brief Set the controllers integral gain.
     * @param ki The integral gain.
     */
    inline void set_integral_gain(const Eigen::Ref<const VectorXd> ki) {
        m_ki = ki;
    }

protected:

    /**
     * @brief Create a new PID controller.
     * @param configuration Configuration of the pid controller.
     */
    PID(const Configuration &configuration);

    /// Proportional gain.
    VectorXd m_kp;

    /// Derivative gain.
    VectorXd m_kd;

    /// Integral gain.
    VectorXd m_ki;

    /// Minimum control output.
    VectorXd m_minimum;

    /// Maximum control output.
    VectorXd m_maximum;

    /// The reference state.
    VectorXd m_reference;

    /// Cached storage for the error signal.
    VectorXd m_error;

    /// The previously calculated error, for derivative calculation.
    VectorXd m_last_error;

    /// The cumulative error, for integral calculation.
    VectorXd m_cumulative_error;

    /// The control action of the last update.
    VectorXd m_saturation;

    /// The control action of the last update.
    VectorXd m_control;

    /// The time of the last update.
    double m_last_time;
};

/**
 * @brief A pid controller for a quaternion and xzx' convention euler angles.
 */
class QuaternionPID : public PID
{
public:

    struct Configuration {

        /// Proportional gain matrix.
        VectorXd kp;

        /// Derivative gain matrix.
        VectorXd kd;

        /// Integral gain matrix.
        VectorXd ki;

        /// The minimum control output.
        VectorXd minimum;

        /// The maximum control output.
        VectorXd maximum;

        /// The initial reference of the pid controller.
        VectorXd reference;

        /// The initial time.
        double initial_time;

        /// JSON conversion for quaternion pid controller.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            kp, kd, ki, minimum, maximum, reference
        )
    };

    /**
     * @brief An approximation for the characteristics of a human attempting to
     * orientate an object by applying torque.
     */
    static inline const Configuration HUMAN_ORIENTATION_CONTROL {
        .kp = Vector3d(500.0, 500.0, 500.0),
        .kd = Vector3d(50.0, 50.0, 50.0),
        .ki = Vector3d(0.0, 0.0, 0.0),
        .minimum = Eigen::Vector3d(-10000.0, -10000.0, -10000.0),
        .maximum = Eigen::Vector3d(10000.0, 10000.0, 10000.0),
        .reference = Eigen::Vector3d::Zero()
    };

    /**
     * @brief Create a quaternion pid controller.
     * 
     * @param configuration The configuration of the pid controller.
     * @return A pointer to the pid controller on success or nullptr on failure.
     */
    static std::unique_ptr<QuaternionPID> create(const Configuration &configuration);

    /**
     * @brief Update the control to apply to the current observed orientation.
     * 
     * @warning The quaternion must be normalised.
     * 
     * @param quaternion The observed orientation of the system.
     * @param time The current time in seconds.
     */
    void update(const Eigen::Ref<const VectorXd> quaternion, double time) override;

    /**
     * @brief Update the controller with an observed quaternion.
     * 
     * @warning The quaternion must be normalised.
     * 
     * @param quaternion The observed quaternion.
     * @param time The current time in seconds.
     */
    void update(const Quaterniond &quaternion, double time);

    inline void set_reference(const Quaterniond &quaternion) {
        
    }

private:

    QuaternionPID(const PID::Configuration &pid);

};

} // namespace controller
