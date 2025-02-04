#include "controller/qp.hpp"

// QuadraticProgram::SparseMatrix::SparseMatrix(Eigen::Ref<Eigen::MatrixXd> m)
//     : columns()
//     , rows()
//     , data()
// {
//     for (auto col : m.colwise()) {
//         columns.push_back(rows.size());
//         for (int i = 0; i < col.size(); i++) {
//             if (col[i] != 0.0) {
//                 rows.push_back(i);
//                 data.push_back(col[i]);
//             }
//         }
//     }
//     columns.push_back(data.size());

//     csc_set_data(
//         &matrix,
//         m.rows(),
//         m.cols(),
//         data.size(),
//         data.data(),
//         rows.data(),
//         columns.data()
//     );
// }

std::unique_ptr<QuadraticProgram> QuadraticProgram::create(
    int optimisation_variables,
    int constraints
) {
    // Allocate the quadratic program.
    auto qp = std::unique_ptr<QuadraticProgram>(new QuadraticProgram());

    // Construct the quadratic and scalar costs.
    auto hessian = Eigen::MatrixXd::Zero(optimisation_variables, optimisation_variables);
    auto scale = Eigen::MatrixXd::Zero(constraints, constraints);

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
