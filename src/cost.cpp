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
    , m_point(1.0, 1.0, 1.0)
{}

double Cost::step(
    const State & state,
    const Control & /*control */,
    double /*dt */
) {
    m_model->update(state);
    auto [position, orientation] = m_model->end_effector();
    return (position - Eigen::Vector3d(1.0, 1.0, 1.0)).norm();
}

void Cost::reset()
{
    // Unused
}

double Cost::cost()
{
    return 0.0;
}
