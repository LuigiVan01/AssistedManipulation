#include "cost.hpp"

#include <iostream>
#include <random>

std::shared_ptr<Cost> Cost::create(const std::string &urdf)
{
    auto model = FrankaRidgeback::Model::create(urdf);
    if (!model) {
        std::cout << "Failed to create dynamics model." << std::endl;
        return nullptr;
    }

    return std::shared_ptr<Cost>(new Cost(std::move(model)));
}

Cost::Cost(std::unique_ptr<FrankaRidgeback::Model> &&model)
    : m_model(std::move(model))
{}

double Cost::step(
    const Eigen::VectorXd & s,
    const Eigen::VectorXd & /*control */,
    double /*dt */
) {
    const FrankaRidgeback::State &state = s;

    m_model->update(state);
    auto [position, orientation] = m_model->end_effector();
    return state.arm_position().norm();
}
