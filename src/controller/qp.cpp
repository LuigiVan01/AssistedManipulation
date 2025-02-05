#include "controller/qp.hpp"

SparseMatrix::SparseMatrix(const Eigen::Ref<Eigen::MatrixXd> m)
    : columns()
    , rows()
    , data()
{
    for (auto col : m.colwise()) {
        columns.push_back(rows.size());
        for (int i = 0; i < col.size(); i++) {
            if (col[i] != 0.0) {
                rows.push_back(i);
                data.push_back(col[i]);
            }
        }
    }
    columns.push_back(data.size());

    csc_set_data(
        hessian_csc,
        m.rows(),
        m.cols(),
        data.size(),
        data.data(),
        rows.data(),
        columns.data()
    );

    settings=(OSQPSettings *)malloc(sizeof(OSQPSettings));
    if (settings) 
        osqp_set_default_settings(settings);
}

void SparseMatrix::call_solver(OSQPInt n, OSQPInt m)
{

    // Create an empty constraint matrix A (n x n)
    OSQPCscMatrix* A =(OSQPCscMatrix*) malloc(sizeof(OSQPCscMatrix));
    OSQPFloat* A_x = (OSQPFloat*)malloc(0 * sizeof(OSQPFloat));  // No non-zero elements
    OSQPInt* A_i = (OSQPInt*)malloc(0 * sizeof(OSQPInt));
    OSQPInt* A_p = (OSQPInt*)malloc((n + 1) * sizeof(OSQPInt));
    for(int i = 0; i <= n; i++) A_p[i] = 0;
    csc_set_data(A, n, n, 0, A_x, A_i, A_p);

    // Create bounds vectors with infinite values
    OSQPFloat* l = (OSQPFloat*)malloc(n * sizeof(OSQPFloat));
    OSQPFloat* u = (OSQPFloat*)malloc(n * sizeof(OSQPFloat));
    for(int i = 0; i < n; i++) {
        l[i] = -OSQP_INFTY;  // Lower bound is negative infinity
        u[i] = OSQP_INFTY;   // Upper bound is positive infinity
    }
    exitflag=osqp_setup(&solver, hessian_csc, linear_terms, A, l, u, n, n, settings);
    if (!exitflag) {
        exitflag = osqp_solve(solver);
        solution=solver->solution;
    }
    else {
        std::cerr << "failed to initialise osqp solver" << std::endl;
    }
    osqp_cleanup(solver);
}

std::unique_ptr<QuadraticProgram> QuadraticProgram::create(
    int optimisation_variables,
    int constraints
) {

    auto qp = std::make_unique<QuadraticProgram>();


    // Construct the quadratic and scalar costs.
    qp->hessian = Eigen::MatrixXd::Zero(
        optimisation_variables, 
        optimisation_variables
    );
    
    //qp->linear = Eigen::VectorXd::Zero(optimisation_variables);

    qp->constraint_matrix = Eigen::MatrixXd::Zero(constraints, constraints);

    // Allocate the quadratic program.
    

    

    // for (int i = 0; i < configuration.objectives.size(); i++) {
    //     Objective &objective = configuration.objectives[i];

    //     qp->m_linear_cost.push_back(objective.linear);

    //     hessian.row(i) = Eigen::Map<Eigen::VectorXd>(
    //         objective.hessian.data(),
    //         objective.hessian.size()
    //     );
    // }

    // for (int i = 0; i < configuration.constraints.size(); i++) {
    //     Constraint &constraint = configuration.constraints[i];

    //     qp->m_lower.push_back(constraint.lower);
    //     qp->m_upper.push_back(constraint.upper);

    //     scale.row(i) = Eigen::Map<Eigen::VectorXd>(
    //         constraint.scale.data(),
    //         constraint.scale.size()
    //     );
    // }

    // // Convert to 
    // qp->m_quadratic_cost = std::make_unique<SparseMatrix>(hessian);
    // qp->m_scale = std::make_unique<SparseMatrix>(scale);

    // OSQPSettings settings;
    // osqp_set_default_settings(&settings);

    // OSQPSolver *solver = nullptr;
    // auto failure = osqp_setup(
    //     &solver,
    //     qp->m_quadratic_cost->get(),
    //     qp->m_linear_cost.data(),
    //     qp->m_scale->get(),
    //     qp->m_lower.data(),
    //     qp->m_upper.data(),
    //     configuration.constraints.size(),
    //     configuration.variables,
    //     &configuration.settings
    // );

    // if (failure) {
    //     std::cerr << "failed to initialise osqp solver" << std::endl;
    //     return nullptr;
    // }

    // qp->m_solver = OSQPSolverPointer(solver, osqp_cleanup);

    return qp;
}

/**
 * @brief Solve the quadratic optimsation problem for the state x.
 * 
 * @param x 
 * @return Eigen::VectorXd 
 */
// inline Eigen::VectorXd solve(Eigen::Ref<Eigen::VectorXd> x)
// {

// }
