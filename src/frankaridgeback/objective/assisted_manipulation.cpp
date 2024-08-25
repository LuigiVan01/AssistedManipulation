#include "frankaridgeback/objective/assisted_manipulation.hpp"

#include <iostream>

std::unique_ptr<AssistedManipulation> AssistedManipulation::create(
    const Configuration &configuration
) {
    auto model = FrankaRidgeback::Model::create(std::move(configuration.model));
    if (!model) {
        std::cout << "failed to create dynamics model." << std::endl;
        return nullptr;
    }

    return std::unique_ptr<AssistedManipulation>(
        new AssistedManipulation(std::move(model), configuration)
    );
}

AssistedManipulation::AssistedManipulation(
    std::unique_ptr<FrankaRidgeback::Model> &&model,
    const Configuration &configuration
  ) : m_model(std::move(model))
    , m_space_jacobian(6, FrankaRidgeback::DoF::JOINTS)
    , m_configuration(configuration)
    , m_cost(0.0)
    , m_power_cost(0.0)
    , m_manipulability_cost(0.0)
    , m_joint_cost(0.0)
    , m_reach_cost(0.0)
{}

double AssistedManipulation::get(
    const Eigen::VectorXd & s,
    const Eigen::VectorXd & c,
    double /*time */
) {
    const FrankaRidgeback::State &state = s;
    const FrankaRidgeback::Control &control = c;

    m_model->set(state);

    if (m_configuration.enable_minimise_power)
        m_power_cost = power_cost(state, control);

    if (m_configuration.enable_maximise_manipulability)
        m_manipulability_cost = manipulability_cost();

    if (m_configuration.enable_joint_limit)
        m_joint_cost = joint_limit_cost(state);

    if (m_configuration.enable_reach_limit)
        m_reach_cost = reach_cost();

    if (m_configuration.enable_variable_damping)
        m_variable_damping_cost = variable_damping_cost(state);

    m_cost = (
        m_power_cost +
        m_manipulability_cost +
        m_joint_cost +
        m_reach_cost
    );

    return m_cost;
}

double AssistedManipulation::power_cost(
    const FrankaRidgeback::State &state,
    const FrankaRidgeback::Control &control
) {
    // Minimise power.
    const auto &maximum = m_configuration.maximum_power;

    double power = (
        state.arm_velocity().transpose() * control.arm_torque()
    ).eval().value();

    if (power < maximum.limit)
        return 0.0;

    return (
        maximum.constant_cost +
        std::max(0.0, maximum.quadratic_cost * (power - maximum.limit))
    );
}

double AssistedManipulation::manipulability_cost()
{
    const auto &minimum = m_configuration.minimum_manipulability;

    pinocchio::computeJointJacobians(*m_model->get_model(), *m_model->get_data());
    m_space_jacobian = m_model->get_data()->J * m_model->get_data()->J.transpose();

    // Value proportional to the volumne of the manipulability ellipsoid.
    double ellipsoid_volume = std::sqrt(m_space_jacobian.determinant());

    // Clip to a small value to prevent division by zero on singularity.
    if (ellipsoid_volume < 1e-10)
        ellipsoid_volume = 1e-10;

    return minimum.quadratic_cost * std::pow(1 / ellipsoid_volume, 2);
}

double AssistedManipulation::joint_limit_cost(const FrankaRidgeback::State &state)
{
    double cost = 0.0;

    for (size_t i = 0; i < FrankaRidgeback::DoF::JOINTS; i++) {
        const auto &lower = m_configuration.lower_joint_limit[i];
        const auto &upper = m_configuration.upper_joint_limit[i];

        double position = state.position()(i);

        if (position < lower.limit) {
            cost += (
                lower.constant_cost +
                lower.quadratic_cost * std::pow(lower.limit - position, 2)
            );
        }
        else if (position > upper.limit) {
            cost += (
                upper.constant_cost +
                lower.quadratic_cost * std::pow(position - upper.limit, 2)
            );
        }
    }

    return cost;
}

double AssistedManipulation::reach_cost()
{
    const auto &max = m_configuration.maximum_reach;
    const auto &min = m_configuration.minimum_reach;

    // Distance of end effector to base frame.
    double reach = 0.0; // TODO

    if (reach > max.limit) {
        return (
            max.constant_cost +
            max.quadratic_cost * std::pow(max.limit - reach, 2)
        );
    }
    else if (reach < min.limit) {
        return (
            min.constant_cost +
            min.quadratic_cost * std::pow(reach - min.limit, 2)
        );
    }

    return 0.0;
}

double AssistedManipulation::variable_damping_cost(const FrankaRidgeback::State &state)
{
    // double velocity = m_model->end_effector_velocity().norm();

    // double expected_damping = (
    //     m_configuration.variable_damping_maximum *
    //     std::exp(-m_configuration.variable_damping_dropoff * velocity)
    // );

    // // The expected resistive force against the external force.
    // double expected_inertia = expected_damping * velocity;

    // double inertia = m_model->get_data().Jm_model->get_data()->M
    // // Subtract the 
    // double residual_force = (
    //     state.end_effector_force().norm() - expected_damping * velocity
    // );

    // // // F = c * x_dot therefore c = F \ x_dot
    // double damping = state.end_effector_torque().norm() / velocity;

    // m_variable_damping_cost.quadratic * ;

    return 0.0;
}
