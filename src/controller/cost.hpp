#pragma once

#include <cmath>

#include "controller/json.hpp"

/**
 * @brief A generic objective function for evaluating a single variable.
 * 
 * The quadratic cost performs the role of a quadratic gradient function
 * during optimisation.
 * 
 * TODO: Could add an inline function calculating the cost as
 * c(x) = constant_cost + linear_cost * x + quadratic_cost * x^2
 * to simplify writing it out in every part of the objective function.
 */
struct QuadraticCost {

    /// The limit.
    double limit = 0.0;

    /// Constant cost incurred when limit is breached.
    double constant_cost = 0.0;

    /// Cost incurred proportional to the limit.
    double linear_cost = 0.0;

    /// Cost incurred proportional to the square of the limit.
    double quadratic_cost = 0.0;

    /**
     * @brief Evaluate the quadratic cost.
     * @returns The cost.
     */
    inline double operator()(double value) const {
        return (
            constant_cost +
            linear_cost * std::fabs(value) +
            quadratic_cost * value * value
        );
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        QuadraticCost,
        limit, linear_cost, constant_cost, quadratic_cost
    )
};
