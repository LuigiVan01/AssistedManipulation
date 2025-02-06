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
class OSQPFormulation {

    public:
        /**
         * @brief Creates an osqp sparse matrix from an eigen matrix.
         * @param matrix The matrix to construct an OSQP sparse matrix from.
         */


        

        void call_solver(OSQPInt n, OSQPInt m);

        
        void SparseMatrix(const Eigen::Ref<Eigen::MatrixXd> m);


        /// OSQP matrix data.
        OSQPCscMatrix* hessian_csc=(OSQPCscMatrix *)malloc(sizeof(OSQPCscMatrix));;
        OSQPCscMatrix* constraints_matrix_csc=(OSQPCscMatrix *)malloc(sizeof(OSQPCscMatrix));;

        OSQPFloat* linear_terms;

        OSQPSettings* settings;

        OSQPSolver*   solver;

        OSQPSolution* solution;

        OSQPInt exitflag;

        std::vector<OSQPInt> csc_columns;
        std::vector<OSQPInt> csc_rows;
        std::vector<OSQPFloat> csc_data;


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

    

    /// Minimise 1/2 * x * hessian * x
    Eigen::MatrixXd hessian;

    /// Minimise linear * x
    Eigen::VectorXd linear;
    
    // lower <= constraint_matrix * x <= upper.
    Eigen::MatrixXd constraint_matrix;

    /// Lower bound constraints. Ensure lower <= Ax.
    Eigen::VectorXd lower;

    /// Upper boundary constraints. Ensure upper >= Ax.
    Eigen::VectorXd upper;

    std::unique_ptr<OSQPFormulation> formulation;


    // using OSQPSolverPointer = std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)>;



    QuadraticProgram(int optimisation_variables, int constraints);


};
 