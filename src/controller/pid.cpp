#include "controller/pid.hpp"

#include <iostream>
#include <limits>

namespace controller {

std::unique_ptr<PID> PID::create(const Configuration &configuration)
{
    bool equal_dimensions = (
        configuration.kp.size() == configuration.n &&
        configuration.kp.size() == configuration.n &&
        configuration.kd.size() == configuration.n &&
        configuration.ki.size() == configuration.n &&
        configuration.minimum.size() == configuration.n &&
        configuration.reference.size() == configuration.n
    );

    if (!equal_dimensions) {
        std::cerr << "pid parameters must have the same dimension "
                    << configuration.n << ". Got" << std::endl;
        std::cerr << "    kp.size() = " << configuration.kp.size() << std::endl;
        std::cerr << "    kd.size() = " << configuration.kd.size() << std::endl;
        std::cerr << "    ki.size() = " << configuration.ki.size() << std::endl;
        std::cerr << "    minimum.size() = " << configuration.minimum.size() << std::endl;
        std::cerr << "    maximum.size() = " << configuration.maximum.size() << std::endl;
        std::cerr << "    reference.size() = " << configuration.reference.size() << std::endl;
        return nullptr;
    }

    return std::unique_ptr<PID>(new PID(configuration));
}

PID::PID(const Configuration &configuration)
    : m_kp(configuration.kp)
    , m_kd(configuration.kd)
    , m_ki(configuration.ki)
    , m_minimum(configuration.minimum)
    , m_maximum(configuration.maximum)
    , m_reference(configuration.reference)
    , m_error(Eigen::VectorXd::Zero(configuration.n))
    , m_last_error(Eigen::VectorXd::Zero(configuration.n))
    , m_cumulative_error(Eigen::VectorXd::Zero(configuration.n))
    , m_saturation(Eigen::VectorXd::Zero(configuration.n))
    , m_control(Eigen::VectorXd::Zero(configuration.n))
    , m_last_time(configuration.initial_time)
    , m_derivative_invalid(true)
{}

void PID::update(const Eigen::Ref<const Eigen::VectorXd> state, double time)
{
    // Time must be monotonically increasing. Also waits until dt
    // calculations are valid.
    if (time <= m_last_time)
        return;

    double dt = time - m_last_time;
    VectorXd error = m_reference - state;

    // Ensure enough data to calculate derivative.
    if (m_derivative_invalid) {
        m_last_error = error;
        m_last_time = time;
        m_derivative_invalid = false;
        return;
    }

    // Runge kutta? Inaccuracy of small number division?

    // Calculate control and saturate.
    m_control = (
        m_kp.cwiseProduct(error) +
        m_kd.cwiseProduct(error - m_last_error) / dt +
        m_ki.cwiseProduct(m_cumulative_error)
    ).cwiseMin(m_maximum).cwiseMax(m_minimum);

    // Calculate saturation per degree of freedom.
    m_saturation = (
        m_control.array() >= m_maximum.array() ||
        m_control.array() <= m_minimum.array()
    ).cast<double>();

    // If saturation is zero then accumulate error (anti-windup).
    m_cumulative_error += error.cwiseProduct(
        m_saturation.cwiseEqual(VectorXd::Zero(m_saturation.size())).cast<double>()
    ) * dt;

    m_last_error = error;
    m_last_time = time;
}

std::unique_ptr<QuaternionPID> QuaternionPID::create(
    const Configuration &configuration
) {
    PID::Configuration pid {
        .n = 3,
        .kp = configuration.kp,
        .kd = configuration.kd,
        .ki = configuration.ki,
        .minimum = configuration.minimum,
        .maximum = configuration.maximum,
        .reference = configuration.reference,
        .initial_time = configuration.initial_time
    };

    return std::unique_ptr<QuaternionPID>(
        new QuaternionPID(pid)
    );
}

QuaternionPID::QuaternionPID(const PID::Configuration &pid)
    : PID(pid)
{}

void QuaternionPID::update(const Eigen::Ref<const Eigen::VectorXd> state, double time)
{
    assert(state.size() == 4);
    Quaterniond quaternion {(Vector4d)state};
    update(quaternion.normalized(), time);
}

void QuaternionPID::update(const Quaterniond &quaternion, double time)
{

}

} // namespace controller
