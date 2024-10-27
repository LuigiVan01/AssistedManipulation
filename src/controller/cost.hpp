#pragma once

#include <cmath>

#include "controller/json.hpp"

/**
 * @brief A generic objective function for evaluating a single variable.
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

/**
 * @brief An inverse barrier function which increases with cost until an upper
 * bound, clamped to a maximum cost. No lower bound.
 */
struct RightInverseBarrierFunction {

    /// The upper value of the barrier.
    double upper_bound;

    /// The scaling factor.
    double scale;

    /// The maximum value to clamp the barrier function to.
    double maximum_cost = 1e10;

    /**
     * @brief Calculate the cost of the barrier function.
     */
    inline double operator()(double value) const
    {
        if (value >= upper_bound)
            return NAN;
        return std::min(scale / (upper_bound - value), maximum_cost);
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        RightInverseBarrierFunction,
        upper_bound, scale, maximum_cost
    )
};

/**
 * @brief An inverse barrier function which increases with cost until a lower
 * bound, clamped to a maximum cost. No upper bound.
 */
struct LeftInverseBarrierFunction {

    /// The barrier.
    double lower_bound;

    /// Scaling factor of the 
    double scale;

    /// The maximum value to clamp the barrier function to.
    double maximum_cost = 1e10;

    /**
     * @brief Calculate the cost of the barrier function.
     */
    inline double operator()(double value) const
    {
        if (value <= lower_bound)
            return NAN;
        return std::min(scale / (value - lower_bound), maximum_cost);
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        LeftInverseBarrierFunction,
        lower_bound, scale, maximum_cost
    )
};

/**
 * @brief An logarithmic barrier function which increases with cost until an
 * upper bound, clamped to a maximum cost. No lower bound.
 */
struct UpperLogarithmicBarrierFunction {

    /// The barrier.
    double upper_bound;

    /// The scaling factor of the function.
    double scale;

    /// The value offset.
    double offset;

    /// The maximum value to clamp the barrier function to.
    double maximum_cost = 1e10;

    /**
     * @brief Calculate the cost of the barrier function.
     */
    inline double operator()(double value) const
    {
        if (value >= upper_bound)
            return maximum_cost;
        return std::min(scale * (-std::log10(-value + upper_bound) + offset), 0.0);
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        UpperLogarithmicBarrierFunction,
        upper_bound, scale, offset, maximum_cost
    )
};

/**
 * @brief An logarithmic barrier function which increases with cost until a
 * lower bound, clamped to a maximum cost. No upper bound.
 */
struct LowerLogarithmicBarrierFunction {

    /// The barrier.
    double lower_bound;

    /// The scaling factor of the function.
    double scale;

    /// The value offset.
    double offset;

    /// The maximum value to clamp the barrier function to.
    double maximum_cost = 1e10;

    /**
     * @brief Calculate the cost of the barrier function.
     */
    inline double operator()(double value) const
    {
        if (value <= lower_bound)
            return maximum_cost;
        return std::min(scale * (-std::log10(value - lower_bound) + offset), 0.0);
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        LowerLogarithmicBarrierFunction,
        lower_bound, scale, offset, maximum_cost
    )
};
