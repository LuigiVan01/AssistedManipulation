#pragma once

#include <nlohmann/json.hpp>

#include "controller/mppi.hpp"
#include "controller/pid.hpp"

using json = nlohmann::json;

namespace nlohmann {

template<typename T>
struct adl_serializer<std::optional<T>>
{
    static void to_json(json& j, const std::optional<T>& opt) {
        if (opt) {
            j = *opt;
        } else {
            j = nullptr;
        }
    }

    static void from_json(const json& j, std::optional<T>& opt) {
        if (j.is_null()) {
            opt = std::nullopt;
        } else {
            opt = j.get<T>();
        }
    }
};

} // namespace nholmann

template <typename Derived>
void to_json(json &j, const Eigen::MatrixBase<Derived> &matrix)
{
    j = json::array();

    for (int i = 0; i < matrix.rows(); i++) {
        auto row = json::array();

        for (int j = 0; j < matrix.cols(); i++)
            row.push_back(matrix(i, j));

        j.push_back(row);
    }
}

template <typename Derived>
void from_json(const json &j, Eigen::MatrixBase<Derived> &matrix)
{
    using T = typename MatrixBase<Derived>::Scalar;

    for (int i = 0; i < matrix.rows(); i++) {
        for (int j = 0; j < matrix.cols(); j++) {
            matrix(i, j) = j.at(i).at(j).get<T>();
        }
    }
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    mppi::Trajectory::Configuration::Smoothing,
    window, order
)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    mppi::Trajectory::Configuration,
    initial_state, rollouts, keep_best_rollouts, time_step, horison,
    gradient_step, cost_scale, cost_discount_factor, covariance, control_bound,
    control_min, control_max, control_default, smoothing, threads
)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    controller::PID::Configuration,
    state_dof, control_dof, kp, kd, ki, minimum, maximum, reference, time
)
