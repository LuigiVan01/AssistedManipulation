#pragma once

#include <random>

#include "controller/eigen.hpp"

/**
 * @brief A multivariate gaussian sampler.
 * 
 * https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
 */
class Gaussian
{
public:

    /**
     * @brief Create a new multivariate gaussian.
     * 
     * The covariance matrix must be square, and the mean must have length
     * equal to the number of covariance rows.
     * 
     * @param mean The mean of each gaussian.
     * @param covariance The covariance matrix of the distribution.
     */
    inline Gaussian(const VectorXd &mean, const MatrixXd &covariance)
        : m_mean(mean)
        , m_generator()
        , m_distribution(0, 1)
    {
        set_covariance(covariance);
    }

    /**
     * @brief Create a new multivariate gaussian with zero mean.
     * 
     * The covariance matrix must be square.
     * 
     * @param covariance The covariance matrix of the distribution.
     */
    inline Gaussian(const MatrixXd &covariance)
        : Gaussian(VectorXd::Zero(covariance.rows()), covariance)
    {}

    /**
     * @brief Set the covariance of the distribution.
     * @param covariance The covariance matrix of the distribution.
     */
    inline void set_covariance(const MatrixXd &covariance)
    {
        Eigen::SelfAdjointEigenSolver<MatrixXd> solver(covariance);
        m_transform = (
            solver.eigenvectors() *
            solver.eigenvalues().cwiseSqrt().asDiagonal()
        );
    }

    /**
     * @brief Set the mean of the distribution.
     * @param mean The mean of the distribution.
     */
    inline void set_mean(const VectorXd &mean)
    {
        m_mean = mean;
    }

    /**
     * @brief Sample the distribution.
     * @returns A vector of values sampled from each gaussian.
     */
    inline VectorXd operator()()
    {
        return m_mean + m_transform * VectorXd(m_mean.size()).unaryExpr(
            [&](double){ return m_distribution(m_generator); }
        );
    }

private:

    /// The mean of each gaussian.
    VectorXd m_mean;

    /// Transformation matrix from N(0, 1) noise to the multivariate noise.
    MatrixXd m_transform;

    /// The pseudo-random number generator.
    std::mt19937 m_generator;

    /// A N(0, 1) gaussian distribution single values from.
    std::normal_distribution<double> m_distribution;
};
