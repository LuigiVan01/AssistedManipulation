#include "controller/qp.hpp"


void OSQPFormulation::SparseMatrix(const Eigen::Ref<Eigen::MatrixXd> m)
{

    csc_columns.clear();
    csc_rows.clear();
    csc_data.clear();
        
    for (auto col : m.colwise()) {
        csc_columns.push_back(csc_rows.size());
        for (int i = 0; i < col.size(); i++) {
            if (col[i] != 0.0) {
                csc_rows.push_back(i);
                csc_data.push_back(col[i]);
            }
        }
    }
    csc_columns.push_back(csc_data.size());

    // Print the CSC format arrays
    std::cout << "CSC Format Arrays:" << std::endl;

    // Print columns array (column pointers)
    std::cout << "Columns (column pointers): [";
    for (size_t i = 0; i < csc_columns.size(); ++i) {
        std::cout << csc_columns[i];
        if (i < csc_columns.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Print rows array (row indices)
    std::cout << "Rows (row indices): [";
    for (size_t i = 0; i < csc_rows.size(); ++i) {
        std::cout << csc_rows[i];
        if (i < csc_rows.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Print data array (values)
    std::cout << "Data (values): [";
    for (size_t i = 0; i < csc_data.size(); ++i) {
        std::cout << csc_data[i];
        if (i < csc_data.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Print matrix dimensions and number of non-zero elements
    std::cout << "Matrix dimensions: " << m.rows() << " x " << m.cols() << std::endl;
    std::cout << "Number of non-zero elements: " << csc_data.size() << std::endl;
    
    // hessian_csc->m     = m.rows();
    // hessian_csc->n     = m.cols();
    // hessian_csc->nz   = -1;
    // hessian_csc->nzmax = csc_data.size();
    // hessian_csc->x     = csc_data.data();
    // hessian_csc->i     = csc_rows.data();
    // hessian_csc->p     = csc_columns.data();

    // hessian_csc->p = csc_columns.data();

    csc_set_data(
        hessian_csc,
        m.rows(),
        m.cols(),
        csc_data.size(),
        csc_data.data(),
        csc_rows.data(),
        csc_columns.data()
    );

    printf("Hessian Matrix (CSC format) properties:\n");
    printf("Number of rows (m): %d\n", hessian_csc->m);
    printf("Number of columns (n): %d\n", hessian_csc->n);
    printf("Maximum nonzeros (nzmax): %d\n", hessian_csc->nzmax);
    printf("Actual nonzeros (nz): %d\n", hessian_csc->nz);

    printf("\nColumn pointers (p):\n");
    for (OSQPInt j = 0; j <= hessian_csc->n; j++) {
        printf("p[%d] = %d\n", j, hessian_csc->p[j]);
    }

    printf("\nRow indices (i) and values (x):\n");
    for (OSQPInt k = 0; k < hessian_csc->p[hessian_csc->n]; k++) {
        printf("i[%d] = %d, x[%d] = %f\n", k, hessian_csc->i[k], k, hessian_csc->x[k]);
    }

    settings=(OSQPSettings *)malloc(sizeof(OSQPSettings));
    if (settings) 
        osqp_set_default_settings(settings);
}

void OSQPFormulation::call_solver(OSQPInt opt_var, OSQPInt constr_row)
{

    // Create an empty constraint matrix A (n x n)
    OSQPCscMatrix* A =(OSQPCscMatrix*) malloc(sizeof(OSQPCscMatrix));
    //OSQPFloat* A_x = (OSQPFloat*)malloc(0 * sizeof(OSQPFloat));  // No non-zero elements
    //OSQPInt* A_i = (OSQPInt*)malloc(0 * sizeof(OSQPInt));
    OSQPInt* A_p = (OSQPInt*)malloc((opt_var + 1) * sizeof(OSQPInt));
    for(int i = 0; i <= opt_var; i++) A_p[i] = 0;
    csc_set_data(A, 1, opt_var, 0, {}, {}, A_p);

    // Create bounds vectors with infinite values
    //OSQPFloat* l = (OSQPFloat*)malloc(n * sizeof(OSQPFloat));
    //OSQPFloat* u = (OSQPFloat*)malloc(n * sizeof(OSQPFloat));
    for(int i = 0; i < opt_var; i++) {
        //l[i] = -OSQP_INFTY;  // Lower bound is negative infinity
        //u[i] = OSQP_INFTY;   // Upper bound is positive infinity
    }

    OSQPFloat* l = nullptr;
    OSQPFloat* u = nullptr;



    // Add debug prints to verify CSC format

    printf("Hessian Matrix (CSC format) properties:\n");
    printf("Number of rows (m): %d\n", hessian_csc->m);
    printf("Number of columns (n): %d\n", hessian_csc->n);
    printf("Maximum nonzeros (nzmax): %d\n", hessian_csc->nzmax);
    printf("Actual nonzeros (nz): %d\n", hessian_csc->nz);

    printf("\nColumn pointers (p):\n");
    for (OSQPInt j = 0; j <= hessian_csc->n; j++) {
        printf("p[%d] = %d\n", j, hessian_csc->p[j]);
    }

    printf("\nRow indices (i) and values (x):\n");
    for (OSQPInt k = 0; k < hessian_csc->p[hessian_csc->n]; k++) {
        printf("i[%d] = %d, x[%d] = %f\n", k, hessian_csc->i[k], k, hessian_csc->x[k]);
    }


    exitflag=osqp_setup(&solver, hessian_csc, linear_terms, A, l, u, 1, 25, settings);
    if (!exitflag) {
        exitflag = osqp_solve(solver);
        solution=solver->solution;
    }
    else {
        std::cerr << "failed to initialise osqp solver" << std::endl;
    }
    osqp_cleanup(solver);
}

QuadraticProgram::QuadraticProgram(int optimisation_variables, int constraints)
    : hessian(Eigen::MatrixXd::Zero(
        optimisation_variables, 
        optimisation_variables))
    , linear(Eigen::VectorXd::Zero(optimisation_variables))
    , constraint_matrix(Eigen::MatrixXd::Zero(constraints, constraints))
    , lower(Eigen::VectorXd::Zero(constraints))
    , upper(Eigen::VectorXd::Zero(constraints))
    , formulation(std::make_unique<OSQPFormulation>())
    {}



// std::unique_ptr<QuadraticProgram> QuadraticProgram::create(
//     int optimisation_variables,
//     int constraints
// ) {

//     //auto qp = std::make_unique<QuadraticProgram>();


//     // Construct the quadratic and scalar costs.
//     qp->hessian = Eigen::MatrixXd::Zero(
//         optimisation_variables, 
//         optimisation_variables
//     );
    
//     //qp->linear = Eigen::VectorXd::Zero(optimisation_variables);

//     qp->constraint_matrix = Eigen::MatrixXd::Zero(constraints, constraints);

//     // Allocate the quadratic program.
    

    

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

//     return qp;
// }

/**
 * @brief Solve the quadratic optimsation problem for the state x.
 * 
 * @param x 
 * @return Eigen::VectorXd 
 */
// inline Eigen::VectorXd solve(Eigen::Ref<Eigen::VectorXd> x)
// {

// }
