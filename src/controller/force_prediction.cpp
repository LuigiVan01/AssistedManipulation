#include "controller/force_prediction.hpp"

#include <functional>
#include <iostream>

std::unique_ptr<KalmanForcePredictor> KalmanForcePredictor::create(
    const Configuration &configuration
) {
    unsigned int dimension = 3 * (configuration.order + 1);

    auto kalman = KalmanFilter::create(KalmanFilter::Configuration{
        .observed_states = 3,
        .states = 3,
        .state_transition_matrix = create_euler_state_transition_matrix(
            configuration.time_step,
            configuration.order
        ),
        .transition_covariance = configuration.transition_variance.asDiagonal(), 
        .observation_matrix = Eigen::MatrixXd::Identity(3, 3),
        .observation_covariance = configuration.observation_variance.asDiagonal(),
        .initial_state = Eigen::Vector3d::Zero()
    });

    if (!kalman) {
        std::cerr << "failed to create force prediction kalman filter" << std::endl;
        return nullptr;
    }

    unsigned int steps = std::ceil(configuration.horison / configuration.time_step);

    auto predictor = std::unique_ptr<KalmanForcePredictor>(new KalmanForcePredictor());
    predictor->m_horison = configuration.horison;
    predictor->m_time_step = configuration.time_step;
    predictor->m_steps = steps;
    predictor->m_last_update = 0.0;
    predictor->m_kalman = std::move(kalman);
    predictor->m_state = Eigen::VectorXd::Zero(dimension);
    predictor->m_prediction = Eigen::MatrixXd(3, steps); // rows, cols
    return predictor;
}

Eigen::MatrixXd KalmanForcePredictor::create_euler_state_transition_matrix(
    double time_step,
    unsigned int order
) {
    std::function<unsigned int(unsigned int)> factorial;
    factorial = [&factorial](unsigned int n) -> unsigned int {
        if (n <= 1)
            return 1;
        return n * factorial(n - 1);
    };

    // Order 0:
    // [1, 0, 0] [ x ]
    // [0, 1, 0] [ y ]
    // [0, 0, 1] [ z ]

    // Order 1:
    // [1, dt, 0,  0, 0,  0] [ x  ]
    // [0,  1, 0,  0, 0,  0] [ dx ]
    // [0,  0, 1, dt, 0,  0] [ y  ]
    // [0,  0, 0,  1, 0,  0] [ dy ]
    // [0,  0, 0,  0, 1, dt] [ z  ]
    // [0,  0, 0,  0, 0,  1] [ dz ]

    // Order 2:
    // [1, dt, 0.5dt^2,  0,  0,       0, 0,  0,       0] [ x   ]
    // [0,  1,      dt,  0,  0,       0, 0,  0,       0] [ dx  ]
    // [0,  0,       1,  0,  0,       0, 0,  0,       0] [ ddx ]
    // [0,  0,       0,  1, dt, 0.5dt^2, 0,  0,       0] [ y   ]
    // [0,  0,       0,  0,  1,      dt, 0,  0,       0] [ dy  ]
    // [0,  0,       0,  0,  0,       1, 0,  0,       0] [ ddy ]
    // [0,  0,       0,  0,  0,       0, 1, dt, 0.5dt^2] [ z   ]
    // [0,  0,       0,  0,  0,       0, 0,  1,      dt] [ dx  ]
    // [0,  0,       0,  0,  0,       0, 0,  0,       1] [ ddz ]

    unsigned int dimension = 3 * (order + 1);

    // Each state (one of x, y or z) has an associated (order x order) square
    // sub-matrix of size (order + 1, order + 1).
    Eigen::MatrixXd matrix {dimension, dimension};

    // Initialise the square sub-matrix for x, y and z.
    for (unsigned int state = 0; state < 3; state++) {
        for (unsigned int order = 0; order < order; order++) {
            unsigned int col = 3 * (order + 1) + order;

            // Loop through the column from bottom to top, filling out the order.
            for (unsigned int i = 0; i <= order; i++) {
                unsigned int row = 3 * state + order - i;
                matrix(row, col) = 1 / factorial(i) * std::pow(time_step, i);
            }
        }
    }

    return matrix;
}

void KalmanForcePredictor::update(Eigen::Vector3d force, double time)
{
    m_last_update = time;
    m_kalman->update(force);

    m_predictor->set_estimation(m_kalman->get_estimation());
    m_predictor->set_covariance(m_kalman->get_covariance());

    // Generate the predicted forces over the horison.
    for (int i = 0; i < m_steps; i++) {
        m_prediction.col(i) = m_predictor->get_estimation();
        m_predictor->predict();
    }
}

void KalmanForcePredictor::update(double time)
{
    m_kalman->predict();
}

Eigen::Vector3d KalmanForcePredictor::predict(double time)
{
    assert(time >= m_last_update);

    static auto get_force = [&](int col) -> Eigen::Vector3d {
        // Todo, rearrange transition matrix to make this
        // m_prediction.topRightCorner(3, 1)
        return Eigen::Vector3d{
            m_prediction(                            0, col),
            m_prediction(    m_prediction.rows() / 3, col),
            m_prediction(2 * m_prediction.rows() / 3, col)
        };
    };

    // If predicting past the horison, return the last predicted force.
    if (time > m_horison)
        return get_force(m_steps - 1);

    // Steps into the current horison.
    double t = (time - m_last_update) / m_time_step;

    // Round down to integer.
    int lower = (int)t;
    int upper = lower + 1;

    // Parameterise between lower and upper.
    t -= lower;

    // Linear interpolation between closest predictions.
    return (1.0 - t) * get_force(lower) + t * get_force(upper);
}
