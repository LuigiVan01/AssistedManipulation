#pragma once

// Controller configuration json conversion utilities.

#include <iostream>
#include <optional>
#include <cstdint>

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>

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

} // namespace nlohmann

namespace Eigen {

// Should be
//
// template<typename Derived>
// void to_json(json &destination, const Eigen::MatrixBase<Derived> &matrix);
//
// template<typename Derived>
// void from_json(const json &source, Eigen::MatrixBase<Derived> &matrix)
//
// See https://github.com/nlohmann/json/issues/3267

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void to_json(json &destination, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &matrix)
{
    destination = json::array();

    for (int i = 0; i < matrix.rows(); i++) {
        auto row = json::array();

        for (int j = 0; j < matrix.cols(); j++)
            row.push_back(matrix(i, j));

        destination.push_back(row);
    }
}

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void from_json(const json &source, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &matrix)
{
    using T = typename Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Scalar;

    if (source.empty())
        matrix.resize(0, 0);

    std::size_t n = source.size();
    std::size_t m = source[0].size();
    matrix.resize(n, m);

    for (std::size_t i = 0; i < n; i++) {
        for (std::size_t j = 0; j < m; j++) {
            matrix(i, j) = source[i][j].get<T>();
        }
    }
}

} // namespace Eigen
