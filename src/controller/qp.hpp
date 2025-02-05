#pragma once

#include <memory>
#include <iostream>

#include <Eigen/Eigen>
#include "osqp.h"


   /**
     * @brief The sparse matrix structure expected by OSQP
     * 
     * Each column is defined as an array of non-zero row indexes and their
     * values.
     * 
     * Each columns data can be accessed by indexing the nonzero row indexes
     * `rows` and their corresponding values in `data` using the indexes in
     * `columns` for each column. There is one index stored for each column.
     * This index points to the first non-zero element of each row. The number
     * of nonzero rows is calculated from the difference to the next column
     * index.
     * 
     * For example, iterating over all the columns:
     * @code
     * for (int col = 0; col < columns.size() - 1; col++) {
     *     std::cout << "column " << col << " has non-zero elements:" << std::endl;
     *     
     *     /// The position at which the column is stored.
     *     int column_index = columns[col];
     * 
     *     int number_of_rows = columns[col + 1] - column_index;
     * 
     *     /// The buffer containing the row indexes.
     *     int *row = &rows[column_index];
     * 
     *     ///The buffer containing the row non-zero elements.
     *     double *values = &data[column_index];
     * 
     *     for (int i = 0; i < number_of_rows; i++)
     *         std::cout << "    row " << rows[i] << " has value " << values[i] << std::endl;
     * }
     * @endcode
     */
class SparseMatrix {

    public:
        /**
         * @brief Creates an osqp sparse matrix from an eigen matrix.
         * @param matrix The matrix to construct an OSQP sparse matrix from.
         */
        SparseMatrix(const Eigen::Ref<Eigen::MatrixXd> m);

        void call_solver(OSQPInt n, OSQPInt m);


        /// OSQP matrix data.
        OSQPCscMatrix* hessian_csc=(OSQPCscMatrix *)malloc(sizeof(OSQPCscMatrix));;

        OSQPFloat* linear_terms;
        /// Indexes into `rows` and `data` for each column.
        std::vector<OSQPInt> columns; 

        /// Indexes into each column that have nonzero data.
        std::vector<OSQPInt> rows;

        /// The non-zero elements of the matrix. Since this is a positive
        /// semi-definite matrix and therefore symmetric, only the upper
        /// triangle is defined.
        std::vector<OSQPFloat> data;

        OSQPSettings* settings;

        OSQPSolver*   solver;

        OSQPSolution* solution;

        OSQPInt exitflag;


    };

    // inline QuadraticProgram()
    //     : m_solver(nullptr, nullptr)
    //     , m_quadratic_cost(nullptr)
    //     , m_scale(nullptr)
    // {}

//     OSQPSolverPointer m_solver;

//     std::unique_ptr<SparseMatrix> m_quadratic_cost;

//     std::vector<OSQPFloat> m_linear_cost;

//     /// One column per variable, one row per constraint, scales state for each
//     /// constraint.
//     std::unique_ptr<SparseMatrix> m_scale;

//     std::vector<OSQPFloat> m_lower;

//     std::vector<OSQPFloat> m_upper;






class QuadraticProgram{
public:

    QuadraticProgram() = default;

    /// Minimise 1/2 * x * hessian * x
    Eigen::MatrixXd hessian;

    //TODO:change
    /// Minimise linear * x
    double* linear = (double*)malloc(20 * sizeof(double));;
    

  
    // lower <= constraint_matrix * x <= upper.
    Eigen::MatrixXd constraint_matrix;

    /// Lower bound constraints. Ensure lower <= Ax.
    Eigen::VectorXd lower;

    /// Upper boundary constraints. Ensure upper >= Ax.
    Eigen::VectorXd upper;

    std::unique_ptr<SparseMatrix> m_hessian_csc;
    std::unique_ptr<SparseMatrix> m_constraint_matrix_csc;

    // struct Configuration {

    /// The number of variables to optimise for.
    //     std::int64_t variables;

    //     std::vector<Objective> objectives;

    /// Constraints applied to the variables.
    //     std::vector<Constraint> constraints;
    // };

    // using OSQPSolverPointer = std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)>;

    /**
     * @brief Creates a quadratic program.
     * 
     * @param configuration 
     * @return std::unique_ptr<QuadraticProgram> 
     */
    std::unique_ptr<QuadraticProgram> create(
        int optimisation_variables,
        int constraints);


};
 