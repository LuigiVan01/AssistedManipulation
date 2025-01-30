#include "controller/forecast.hpp"

#include <limits>
#include <functional>
#include <iostream>

std::unique_ptr<Forecast> Forecast::create(
    const Configuration &configuration
) {
    if (configuration.type == Configuration::Type::LOCF) {
        if (!configuration.locf) {
            std::cerr << "locf forecast selected with no configuration provided" << std::endl;
            return nullptr;
        }

        return LOCFForecast::create(*configuration.locf);
    }

    if (configuration.type == Configuration::Type::AVERAGE) {
        if (!configuration.locf) {
            std::cerr << "average forecast selected with no configuration provided" << std::endl;
            return nullptr;
        }

        return AverageForecast::create(*configuration.average);
    }

    if (configuration.type == Configuration::Type::KALMAN) {
        if (!configuration.kalman) {
            std::cerr << "kalman forecast selected with no configuration provided" << std::endl;
            return nullptr;
        }

        return KalmanForecast::create(*configuration.kalman);
    }

    std::cerr << "unknown forecast type " << configuration.type << "selected" << std::endl;
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
        new AverageForecast(
            configuration.window,
            configuration.states
        )
    );
}

AverageForecast::AverageForecast(double window, unsigned int states)
    : m_mutex()
    , m_window(window)
    , m_buffer()
    , m_average(VectorXd::Zero(states))
    , m_last(0.0)
{}

void AverageForecast::clear_old_measurements(double time)
{
    if (m_buffer.empty())
        return;

    // Find the element that is older than the time window.
    auto it = std::upper_bound(
        m_buffer.begin(),
        m_buffer.end(),
        time - m_window,
        [](double time, const std::pair<double, VectorXd> &element){
            return time < element.first;
        }
    );

    // Erase all elements on range [begin, it) where it points to the first
    // element with time greater than (time - window), excluding the most recent
    // measurement.
    m_buffer.erase(m_buffer.begin(), it);
}

void AverageForecast::update_average()
{
    if (m_buffer.empty()) {
        m_average.setZero();
        return;
    }

    // Update the average.
    VectorXd total = m_buffer[0].second;
    for (int i = 1; i < m_buffer.size(); i++) {
        total += m_buffer[i].second;
    }

    m_average = total / m_buffer.size();
}

void AverageForecast::update(double time)
{
    std::unique_lock lock(m_mutex);
    clear_old_measurements(time);
    update_average();
}

void AverageForecast::update(VectorXd measurement, double time)
{
    std::unique_lock lock(m_mutex);

    // Ignore measurements in the past.
    if (time < m_last)
        return;
    else
        m_last = time;

    m_buffer.emplace_back(time, measurement);
    clear_old_measurements(time);
    update_average();
}

VectorXd AverageForecast::forecast(double time)
{
    std::shared_lock lock(m_mutex);
    return m_average;
}

std::unique_ptr<KalmanForecast> KalmanForecast::create(
    const Configuration &configuration
) {
    // The number of states includes the derivatives of each observed state. For
    // example, observing (x, y) position with a second order model (constant
    // acceleration) has 6 states being x, y, dx, dy, ddx, ddy
    unsigned int states = configuration.observed_states * (configuration.order + 1);

    VectorXd initial_state = VectorXd::Zero(states);
    initial_state.head(configuration.observed_states) = configuration.initial_state;

    KalmanFilter::Configuration kalman_configuration {
        .observed_states = states,
        .states = states,
        .state_transition_matrix = create_euler_state_transition_matrix(
            configuration.time_step,
            configuration.observed_states,
            configuration.order
        ),
        .transition_covariance = create_euler_state_transition_covariance_matrix(
            configuration.variance,
            configuration.observed_states,
            configuration.order
        ),
        .observation_matrix = MatrixXd::Identity(states, states),
        .observation_covariance = MatrixXd::Identity(states, states) * 1e-8,
        .initial_state = initial_state,
        .initial_covariance = MatrixXd::Identity(states, states) * 1e-8
    };

    auto filter = KalmanFilter::create(kalman_configuration);
    auto predictor = KalmanFilter::create(kalman_configuration);

    if (!filter || !predictor ) {
        std::cerr << "failed to create wrench prediction kalman filter" << std::endl;
        return nullptr;
    }

    unsigned int steps = std::ceil(configuration.horison / configuration.time_step) ;

    return std::unique_ptr<KalmanForecast>(
        new KalmanForecast(
            configuration,
            steps,
            0.0,
            std::move(filter),
            std::move(predictor),
            configuration.initial_state
        )
    );
}

KalmanForecast::KalmanForecast(
    const Configuration &configuration,
    unsigned int steps,
    double last_update,
    std::unique_ptr<KalmanFilter> &&filter,
    std::unique_ptr<KalmanFilter> &&predictor,
    VectorXd initial_state
  ) : m_observed_states_size(filter->get_observed_state_size())
    , m_estaimted_states(filter->get_estimated_state_size())
    , m_order(configuration.order)
    , m_horison(configuration.horison)
    , m_time_step(configuration.time_step)
    , m_steps(steps)
    , m_last_update(-configuration.time_step)
    , m_mutex()
    , m_measurement(filter->get_estimated_state_size(), 1) // rows, cols
    , m_filter(std::move(filter))
    , m_predictor(std::move(predictor))
    , m_prediction(m_observed_states_size, steps + 1)
{
    m_measurement.setZero();
}

unsigned int KalmanForecast::factorial(unsigned int n)
{
    if (n <= 1)
        return 1;
    return n * factorial(n - 1);
}

MatrixXd KalmanForecast::create_euler_state_transition_matrix(
    double time_step,
    unsigned int observed_states,
    unsigned int order
) {
    // For example, observed_states = 3:

    // 3 Observed states, order 0:
    // [1, 0, 0] [ x ]
    // [0, 1, 0] [ y ]
    // [0, 0, 1] [ z ]

    // 3 observed states, order 1:
    // [1,  0, 0, dt,  0,  0] [ x ]
    // [0,  1, 0,  0, dt,  0] [ y ]
    // [0,  0, 1,  0,  0, dt] [ z ]
    // [0,  0, 0,  1,  0,  0] [ dx ]
    // [0,  0, 0,  0,  1,  0] [ dy ]
    // [0,  0, 0,  0,  0,  1] [ dz ]

    // 3 observed states, order 2:
    // [1, 0, 0,    dt,  0,  0,   0.5dt^2,       0,       0] [ x ]
    // [0, 1, 0,    0,  dt,  0,         0, 0.5dt^2,       0] [ y ]
    // [0, 0, 1,    0,  0,   dt,        0,       0, 0.5dt^2] [ z ]
    // [0, 0, 0,    1,  0,   0,        dt,       0,       0] [ dx ]
    // [0, 0, 0,    0,  1,   0,         0,      dt,       0] [ dy ]
    // [0, 0, 0,    0,  0,   1,         0,       0,      dt] [ dz ]
    // [0, 0, 0,    0,  0,   0,         1,       0,        0] [ ddx ]
    // [0, 0, 0,    0,  0,   0,         0,       1,        0] [ ddy ]
    // [0, 0, 0,    0,  0,   0,         0,       0,        1] [ ddz ]

    // 2 observed states, order 2:
    // [1, 0, dt,  0, 0.5dt^2,       0] [ x ]
    // [0, 1, 0,  dt,       0, 0.5dt^2] [ y ]
    // [0, 0, 1,   0,       dt,      0] [ dx ]
    // [0, 0, 0,   1,       0,      dt] [ dy ]
    // [0, 0, 0,   0,       1,       0] [ ddx ]
    // [0, 0, 0,   0,       0,       1] [ ddy ]

    // The number of states includes the derivatives of each observed state.
    unsigned int states = observed_states * (order + 1);

    // The state transition matrix.
    MatrixXd matrix {states, states};
    matrix.setZero();

    // For each derivative in sets of observed_state (e.g. 3) from top to bottom.
    for (unsigned int derivative = 0; derivative <= order; derivative++) {

        // For each state in the set of rows.
        for (unsigned int state = 0; state < observed_states; state++) {
            unsigned int row = derivative * observed_states + state;

            // For each column in that row
            for (unsigned int i = 0; i <= order - derivative; i++) {
                unsigned int col = derivative * observed_states + i * observed_states + state;

                matrix(row, col) = 1.0 / (double)factorial(i) * std::pow(time_step, i);
            }
        }
    }

    return matrix;
}

MatrixXd KalmanForecast::create_euler_state_transition_covariance_matrix(
    const VectorXd &variance,
    unsigned int observed_states,
    unsigned int order
) {
    // The number of states includes the derivatives of each observed state.
    unsigned int states = observed_states * (order + 1);
    MatrixXd matrix = MatrixXd::Identity(states, states) * 1e-8;
    return matrix;
}

void KalmanForecast::update(VectorXd measurement, double time)
{
    std::unique_lock lock(m_mutex);

    double dt = time - m_last_update;

    // Wrench derivative computation
    Vector6d delta = (measurement - m_measurement.segment(0, 6)) / dt;

    // Calculate measurement derivatives.
    for (unsigned int i = 1; i <= m_order; i++) {
        // Compute the double wrench derivvative incase of m_order>1
        Vector6d next_delta = (delta - m_measurement.segment(6 * i, 6)) / dt;

        // Set the wrench derivative computation
        m_measurement.segment(6 * i, 6) = delta;
        
        delta = next_delta;
    }

    m_measurement.segment(0, 6) = measurement;

    m_last_update = time;
    // Compute the kalman optimal gain, estimate the current state and next one. 
    // Update the covariance matrix
    m_filter->update(m_measurement);

    // Just set the estimated state and next state
    m_predictor->set_estimation(m_filter->get_estimation());
    // Just set the uodate covariance matrix
    m_predictor->set_covariance(m_filter->get_covariance());

    // The current estimation.
    m_prediction.col(0) = m_predictor->get_estimation().head(m_observed_states_size);

    // Generate the predicted measurement at each future time step over the
    // horison.
    for (unsigned int i = 0; i < m_steps; i++) {
        m_predictor->predict(false);//! Reason about the covariance error update
        m_prediction.col(i + 1) = m_predictor->get_estimation().head(m_observed_states_size);
    }
}

void KalmanForecast::update(double time)
{
    if (time <= m_last_update)
        return;

    // Update the kalman filter using prediction only, and propagate process
    // covariance.
    m_filter->predict();
}

VectorXd KalmanForecast::forecast(double time)
{
    std::shared_lock lock(m_mutex);

    assert(time >= m_last_update);

    // If predicting past the horison, return the last predicted force.
    if (time > m_last_update + m_horison)
        return Vector6d::Zero();

    // Steps into the current horison.
    double t = (time - m_last_update) / m_time_step;

    // Round down to integer.
    int lower = (int)t;
    int upper = lower + 1;

    // Parameterise between lower and upper.
    t -= lower;

    // Linear interpolation between closest predictions.
    return (
        (1.0 - t) * m_prediction.col(lower).topRows(6) +
        t * m_prediction.col(upper).topRows(6)
    );
}
