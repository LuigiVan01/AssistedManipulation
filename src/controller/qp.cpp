#include "controller/qp.hpp"

SparseMatrix::SparseMatrix(Eigen::Ref<Eigen::MatrixXd> m)
{
    for (auto col : m.colwise()) {
        columns.push_back(columns.size() + m.size());
        for (int i = 0; i < col.size(); i++) {
            if (col[i] != 0.0) {
                rows.push_back(i);
                data.push_back(col[i]);
            }
        }
    }

    columns.push_back(data.size());

    csc_set_data(
        &matrix,
        m.rows(),
        m.cols(),
        data.size(),
        data.data(),
        rows.data(),
        columns.data()
    );
}

std::unique_ptr<QuadraticProgram> QuadraticProgram::create(
    Configuration &&configuration
) {
    // Allocate the quadratic program.
    auto qp = std::unique_ptr<QuadraticProgram>(new QuadraticProgram());

    Eigen::MatrixXd P;
    Eigen::MatrixXd A;

    P.conservativeResize(configuration.variables, configuration.variables);
    A.conservativeResize(Eigen::NoChange, configuration.variables);

    OSQPInt next_column_index = 0;
    for (Constraint &constraint : configuration.constraints) {

        // Add the quadratic constraint.
        P.conservativeResize(P.rows() + 1, P.cols());
    }

    // Convert to 
    qp->m_quadratic_cost = std::make_unique<SparseMatrix>(P);
    qp->m_scale = std::make_unique<SparseMatrix>(A);

    OSQPSolver *solver = nullptr;
    auto failure = osqp_setup(
        &solver,
        qp->m_quadratic_cost->get(),
        qp->m_linear_cost.data(),
        qp->m_scale->get(),
        qp->m_lower.data(),
        qp->m_upper.data(),
        configuration.constraints.size(),
        configuration.variables,
        &configuration.settings
    );

    if (failure) {
        std::cerr << "failed to initialise osqp solver" << std::endl;
        return nullptr;
    }

    qp->m_solver = OSQPSolverPointer(solver, osqp_cleanup);

    return qp;
}

/**
 * @brief Solve the quadratic optimsation problem for the state x.
 * 
 * @param x 
 * @return Eigen::VectorXd 
 */
inline Eigen::VectorXd solve(Eigen::Ref<Eigen::VectorXd> x)
{

}