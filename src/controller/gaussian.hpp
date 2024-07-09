#pragma once

#include <Eigen/Eigen>
#include <random>

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
    inline Gaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
        : m_mean(mean)
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
    inline Gaussian(const Eigen::MatrixXd &covariance)
        : Gaussian(Eigen::VectorXd::Zero(covariance.rows()), covariance)
    {}

    /**
     * @brief Set the covariance of the distribution.
     * @param covariance The covariance matrix of the distribution.
     */
    inline void set_covariance(Eigen::MatrixXd const &covariance)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
        m_transform = (
            solver.eigenvectors() *
            solver.eigenvalues().cwiseSqrt().asDiagonal()
        );
    }

    /**
     * @brief Sample the distribution.
     * @returns A vector of values sampled from each gaussian.
     */
    inline Eigen::VectorXd operator()() const {
        return m_mean + m_transform * Eigen::VectorXd(m_mean.size()).unaryExpr(
            [&](double) { return s_distribution(s_generator); }
        );
    }

private:

    /// The mean of each gaussian.
    Eigen::VectorXd m_mean;

    /// Transformation matrix from N(0, 1) noise to the multivariate noise.
    Eigen::MatrixXd m_transform;

    /// The pseudo-random number generator.
    inline static std::mt19937 s_generator {};

    /// A N(0, 1) gaussian distribution single values from.
    inline static std::normal_distribution<double> s_distribution {};
};
