#include "frankaridgeback/objective/assisted_manipulation.hpp"

#include <iostream>

std::unique_ptr<AssistedManipulation> AssistedManipulation::create(
    Configuration &&configuration
) {
    bool lower = (
        configuration.m_lower_joint_limit &&
        configuration.m_lower_joint_limit->size() != FrankaRidgeback::DoF::JOINTS
    );

    bool upper = (
        configuration.m_upper_joint_limit &&
        configuration.m_upper_joint_limit->size() != FrankaRidgeback::DoF::JOINTS
    );

    if (lower) {
        std::cerr << "lower joint limits has incorrect dimension "
                  << configuration.m_lower_joint_limit->size()
                  << " != expected " << FrankaRidgeback::DoF::JOINTS
                  << std::endl;
        return nullptr;
    }

    if (upper) {
        std::cerr << "upper joint limits has incorrect dimension "
                  << configuration.m_upper_joint_limit->size()
                  << " != expected " << FrankaRidgeback::DoF::JOINTS
                  << std::endl;
        return nullptr;
    }

    auto model = FrankaRidgeback::Model::create(std::move(configuration.model));
    if (!model) {
        std::cout << "Failed to create dynamics model." << std::endl;
        return nullptr;
    }

    return std::unique_ptr<AssistedManipulation>(
        new AssistedManipulation(std::move(model), configuration);
    );
}

AssistedManipulation::AssistedManipulation(
    std::unique_ptr<FrankaRidgeback::Model> &&model,
    const Configuration &configuration
  ) : m_model(std::move(model))
    , m_configuration(configuration)
{}

double AssistedManipulation::get(
    const Eigen::VectorXd & s,
    const Eigen::VectorXd & /*control */,
    double /*time */
) {
    double cost = 0.0;
    const FrankaRidgeback::State &state = s;

    // m_model->set(state);
    // auto [position, orientation] = m_model->end_effector();

    // Add joint limit penalisation.
    if (m_configuration.enable_joint_limits) {
        for (size_t i = 0; i < FrankaRidgeback::DoF::JOINTS; i++) {
            if (state.position()(i) < lower_limit[i])
                cost += 1000 + 100000 * std::pow(lower_limit[i] - state(i), 2);

            if (state.position()(i) > upper_limit[i])
                cost += 1000 + 100000 * std::pow(state(i) - upper_limit[i], 2);
        }
    }

    return cost;
}
