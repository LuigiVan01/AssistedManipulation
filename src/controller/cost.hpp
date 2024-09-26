#pragma once

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

    /// The limits to apply.
    double limit;

    /// Constant cost incurred when limit is breached.
    double constant_cost = 1'000;

    /// Cost incurred proportional to the square of how much limit is
    /// breached.
    // double linear_cost = 10'000;

    /// Cost incurred proportional to the square of how much limit is
    /// breached.
    double quadratic_cost = 100'000;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        QuadraticCost,
        limit, constant_cost, quadratic_cost
    )
};
