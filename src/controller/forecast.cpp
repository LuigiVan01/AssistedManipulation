#include "controller/forecast.hpp"

#include <limits>
#include <functional>
#include <iostream>

std::unique_ptr<Forecast> Forecast::create(
    const Configuration &configuration
) {
    if (std::holds_alternative<AverageForecast::Configuration>(configuration.config))
        return AverageForecast::create(std::get<AverageForecast::Configuration>(configuration.config));

    if (std::holds_alternative<KalmanForecast::Configuration>(configuration.config))
        return KalmanForecast::create(std::get<KalmanForecast::Configuration>(configuration.config));

    std::cerr << "unknown force predictor type" << std::endl;
    return nullptr;
}

std::unique_ptr<AverageForecast> AverageForecast::create(
    const Configuration &configuration
) {
    if (configuration.window < 0.0) {
        std::cerr << "prediction window time is negative" << std::endl;
        return nullptr;
    }

    return std::unique_ptr<AverageForecast>(
        new AverageForecast(configuration.dimensions)
    );
}

AverageForecast::AverageForecast(unsigned int dimensions)
    : m_buffer()
    , m_average(1, dimensions)
{
    Eigen::VectorXd initial {1, dimensions};
    initial.setZero();

    // The initial default force is zero. Will be erased on first update.
    m_buffer.emplace_back(std::numeric_limits<double>::min(), initial);
}

void AverageForecast::update(Eigen::VectorXd measurement, double time)
{
    std::unique_lock lock(m_mutex);

    // Find the element that is older than the time window. 
    auto it = std::upper_bound(
        m_buffer.begin(),
        m_buffer.end(),
        time,
        [](double time, const std::pair<double, Eigen::VectorXd> &element){
            return time < element.first;
        }
    );

    // Remove old elements from the buffer. If all elements except the most
    // recent would be erased, then skip. I.e always keep the most recent
    // measurement.
    if (it != m_buffer.end() && it != m_buffer.end() - 1) {
        m_buffer.erase(m_buffer.begin(), it);
    }

    // Calculate the average.
    Eigen::VectorXd total = m_buffer[0].second;
    for (int i = 1; i < m_buffer.size(); i++) {
        total += m_buffer[i].second;
    }

    m_average = total / m_buffer.size();
}

Eigen::VectorXd AverageForecast::forecast(double /* time */)
{
    std::shared_lock lock(m_mutex);
    return m_average;
}

std::unique_ptr<KalmanForecast> KalmanForecast::create(
    const Configuration &configuration
) {
    unsigned int dimension = configuration.dimension * (configuration.order + 1);

    auto kalman = KalmanFilter::create(KalmanFilter::Configuration{
        .observed_states = configuration.dimension,
        .states = configuration.dimension,
        .state_transition_matrix = create_euler_state_transition_matrix(
            configuration.time_step,
            configuration.dimension,
            configuration.order
        ),
        .transition_covariance = configuration.transition_variance.asDiagonal(), 
        .observation_matrix = Eigen::MatrixXd::Identity(configuration.dimension, configuration.dimension),
        .observation_covariance = configuration.observation_variance.asDiagonal(),
        .initial_state = Eigen::VectorXd::Zero(1, configuration.dimension)
    });

    if (!kalman) {
        std::cerr << "failed to create force prediction kalman filter" << std::endl;
        return nullptr;
    }

    unsigned int steps = std::ceil(configuration.horison / configuration.time_step);

    auto predictor = std::unique_ptr<KalmanForecast>(new KalmanForecast());
    predictor->m_horison = configuration.horison;
    predictor->m_time_step = configuration.time_step;
    predictor->m_steps = steps;
    predictor->m_last_update = 0.0;
    predictor->m_kalman = std::move(kalman);
    predictor->m_state = Eigen::VectorXd::Zero(dimension);
    predictor->m_prediction = Eigen::MatrixXd(3, steps); // rows, cols
    return predictor;
}

Eigen::MatrixXd KalmanForecast::create_euler_state_transition_matrix(
    double time_step,
    unsigned int states,
    unsigned int order
) {
    std::function<unsigned int(unsigned int)> factorial;
    factorial = [&factorial](unsigned int n) -> unsigned int {
        if (n <= 1)
            return 1;
        return n * factorial(n - 1);
    };

    // For example, states = 3:

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

    unsigned int dimension = states * (order + 1);

    // Each state (one of x, y or z) has an associated (order x order)
    // triangular sub-matrix of size (order + 1, order + 1).
    Eigen::MatrixXd matrix {dimension, dimension};

    // Initialise the triangular sub-matrix for x, y and z.
    for (unsigned int state = 0; state < states; state++) {

        // The offset of both rows and columns to the state sub-matrix.
        unsigned int offset = state * order;

        // Populate each column of the triangular sub-matrix, being each order.
        for (unsigned int ord = 0; ord < ord; ord++) {
            unsigned int col = offset + ord;

            // Loop through the column of the triangular matrix from bottom to
            // top. Each additional column has an additional element
            // (triangular).
            for (unsigned int i = 0; i <= ord; i++) {
                unsigned int row = offset + ord - i;
                matrix(row, col) = 1 / factorial(i) * std::pow(time_step, i);
            }
        }
    }

    return matrix;
}

void KalmanForecast::update(Eigen::VectorXd force, double time)
{
    std::unique_lock lock(m_mutex);

    m_last_update = time;
    m_kalman->update(force);

    m_filter->set_estimation(m_kalman->get_estimation());
    m_filter->set_covariance(m_kalman->get_covariance());

    // Generate the predicted force over the time horison.
    for (unsigned int i = 0; i < m_steps; i++) {
        m_prediction.col(i) = m_filter->get_estimation();
        m_filter->predict();
    }
}

void KalmanForecast::update(double time)
{
    // Update the kalman filter using prediction only, and propagate process
    // covariance.
    m_kalman->predict();
}

Eigen::VectorXd KalmanForecast::forecast(double time)
{
    std::shared_lock lock(m_mutex);

    assert(time >= m_last_update);

    // If predicting past the horison, return the last predicted force.
    if (time > m_horison)
        return m_prediction.rightCols(1);

    // Steps into the current horison.
    double t = (time - m_last_update) / m_time_step;

    // Round down to integer.
    int lower = (int)t;
    int upper = lower + 1;

    // Parameterise between lower and upper.
    t -= lower;

    // Linear interpolation between closest predictions.
    return (1.0 - t) * m_prediction.col(lower) + t * m_prediction.col(upper);
}
