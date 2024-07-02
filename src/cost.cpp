#include "cost.hpp"

#include <iostream>
#include <random>

std::unique_ptr<Cost> Cost::create(const std::string &urdf)
{
    auto model = FrankaRidgeback::Model::create(urdf);
    if (!model) {
        std::cout << "Failed to create dynamics model." << std::endl;
        return nullptr;
    }

    return std::unique_ptr<Cost>(new Cost(std::move(model)));
}

Cost::Cost(std::unique_ptr<FrankaRidgeback::Model> &&model)
    : m_model(std::move(model))
{}

double Cost::get(
    const Eigen::VectorXd & s,
    const Eigen::VectorXd & /*control */,
    double /*dt */
) {
    const FrankaRidgeback::State &state = s;

    m_model->set(state);
    auto [position, orientation] = m_model->end_effector();

    // Target end effector at point (1.0, 1.0, 1.0)    
    Eigen::Vector3d target = Eigen::Vector3d(1.0, 1.0, 1.0);

    double cost = 100.0 * std::pow((position - target).norm(), 2);

    Eigen::VectorXd lower_limit(FrankaRidgeback::DoF::JOINTS);
    lower_limit <<
        -2.0, -2.0, -6.28,
        -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
        0.5, 0.5;

    Eigen::VectorXd upper_limit(FrankaRidgeback::DoF::JOINTS);
    upper_limit <<
        2.0, 2.0, 6.28,
        2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973,
        0.5, 0.5;

    // Joint limits.
    for (size_t i = 0; i < 10; i++) {
        if (state(i) < lower_limit[i])
            cost += 1000 + 100000 * std::pow(lower_limit[i] - state(i), 2);

        if (state(i) > upper_limit[i])
            cost += 1000 + 100000 * std::pow(state(i) - upper_limit[i], 2);
    }

    // Eigen::Vector3d collision_vector = m_model->offset("panda_link0", "panda_link7");
    // cost += 1000 * std::pow(std::max(0.0, 0.35 - collision_vector.norm()), 2);

    return cost;
}
