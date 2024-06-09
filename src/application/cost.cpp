#include "application/cost.hpp"

#include <iostream>

std::shared_ptr<Cost> Cost::create(const std::string &urdf)
{
    auto model = Model::create(urdf);
    if (!model) {
        std::cout << "Failed to create dynamics model." << std::endl;
        return nullptr;
    }

    return std::shared_ptr<Cost>(new Cost(std::move(model)));
}

Cost::Cost(std::unique_ptr<Model> &&model)
    : m_model(std::move(model))
    , m_point(1.0, 1.0, 1.0)
{}

double Cost::step(
    const Eigen::Matrix<double, StateDoF, 1> & /*state */,
    const Eigen::Matrix<double, ControlDoF, 1> & /*control */,
    double /*dt */
) {
    return 0;
}

void Cost::reset()
{
    // Unused
}

double Cost::cost()
{
    return 0.0;
}