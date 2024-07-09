#include "mppi.hpp"

#include <cmath>
#include <numeric>
#include <span>

namespace mppi {

std::unique_ptr<Trajectory> Trajectory::create(
    Dynamics *dynamics,
    Cost *cost,
    const Configuration &configuration,
    const Eigen::Ref<Eigen::VectorXd> state
) {
    // Copy configuration to make changes without affecting one passed in.
    Configuration configured = configuration;

    // Ensure dynamics and cost expect the same control.
    if (dynamics->control_dof() != cost->control_dof()) {
        std::cerr << "controller dynamics control dof " << dynamics->control_dof()
                  << " != cost control dof " << cost->control_dof() << std::endl;
        return nullptr;
    }

    // Ensure dynamics and cost expect the same state.
    if (dynamics->state_dof() != cost->state_dof()) {
        std::cerr << "controller dynamics state dof " << dynamics->state_dof()
                  << " != cost state dof " << cost->state_dof() << std::endl;
        return nullptr;
    }

    if (configured.control_min.size() != dynamics->control_dof() ||
        configured.control_max.size() != dynamics->control_dof()) {
        std::cerr << "controller maximum and minimum must have length "
                  << dynamics->control_dof() << std::endl;
        return nullptr;
    }

    if (configured.covariance.rows() != configured.covariance.cols()) {
        std::cerr << "controller covariance matrix not square" << std::endl;
        return nullptr;
    }

    if (configured.covariance.rows() != dynamics->control_dof()) {
        std::cerr << "controller sample variance dof " << configured.covariance.rows()
                  << " != dynamics and cost control dof " <<  dynamics->control_dof()
                  << std::endl;
        return nullptr;
    }

    if (configured.rollouts < 1) {
        std::cerr << "trajectory rollouts must be greater than zero" << std::endl;
        return nullptr;
    }

    if (configured.keep_best_rollouts < 0) {
        std::cerr << "trajectory cached rollouts cannot be less than zero" << std::endl;
        return nullptr;
    }

    if (configured.threads <= 0) {
        std::cerr << "trajectory threads must be positive nonzero" << std::endl;
        return nullptr;
    }

    // Count the zero and negative sample in the number of rollouts.
    configured.rollouts += s_static_rollouts;

    // Number of time steps per rollout.
    int steps = std::ceil(configured.horison / configured.time_step);

    return std::unique_ptr<Trajectory>(
        new Trajectory(
            dynamics,
            cost,
            configured,
            state,
            steps
        )
    );
}

Trajectory::Trajectory(
    Dynamics *dynamics,
    Cost *cost,
    const Configuration &configuration,
    const Eigen::Ref<Eigen::VectorXd> state,
    int steps
) noexcept
  : m_configuration(configuration)
  , m_dynamics(configuration.threads)
  , m_cost(configuration.threads)
  , m_futures(configuration.threads)
  , m_gaussian(configuration.covariance)
  , m_steps(steps)
  , m_state_dof(dynamics->state_dof())
  , m_control_dof(dynamics->control_dof())
  , m_rollout_state(state)
  , m_last_rollout_time(0.0)
  , m_rollouts(configuration.rollouts, Rollout(dynamics->control_dof(), steps))
  , m_weights(configuration.rollouts, 1) // (rows, cols) ...
  , m_gradient(dynamics->control_dof(), steps)
  , m_optimal_control_shifted(dynamics->control_dof(), steps)
  , m_optimal_control(dynamics->control_dof(), steps)
  , m_ordered_rollouts(configuration.rollouts - s_static_rollouts)
  , m_thread_pool(configuration.threads)
{
    m_optimal_control.setZero();
    m_weights.setZero();
    m_gradient.setZero();

    // Set the initial gaussian noise to zero.
    for (auto &rollout : m_rollouts) {
        rollout.noise.setZero();
    }

    // Initialise thread data.
    for (unsigned int i = 0; i < configuration.threads; i++) {
        m_dynamics[i] = dynamics->copy();
        m_cost[i] = cost->copy();
    }

    if (configuration.filter) {
        m_filter = SavitzkyGolayFilter(
            steps,
            dynamics->control_dof(),
            configuration.filter->window,
            configuration.filter->order,
            0,
            configuration.time_step
        );
    }
}

void Trajectory::update(const Eigen::Ref<Eigen::VectorXd> state, double time)
{
    m_rollout_state = state;
    m_rollout_time = time;

    sample(time);
    rollout();
    optimise();
}

void Trajectory::sample(double time)
{
    // The number of time steps to shift the optimal control and best sampled
    // trajectories forward by, to align them with the current time.
    m_shift_by = (std::int64_t)((time - m_last_rollout_time) / m_configuration.time_step);

    // The number of columns to shift back.
    m_shifted = m_steps - m_shift_by;

    // Shift the the optimal control to align with the current time.
    m_optimal_control_shifted.leftCols(m_shifted) = m_optimal_control.rightCols(m_shifted).eval();
    m_optimal_control_shifted.rightCols(m_shift_by) = m_optimal_control.rightCols(1).replicate(1, m_shift_by);

    // Reset to random noise if all trajectories are out of date.
    if (m_shift_by >= m_steps) {
        for (std::int64_t index = s_static_rollouts; index < m_configuration.rollouts; ++index) {
            Rollout &rollout = m_rollouts[index];
            for (int i = 0; i < m_steps; i++)
                rollout.noise.col(i) = m_gaussian();
        }
        return;
    }

    // Regenerate indexes of rollouts. 
    std::iota(m_ordered_rollouts.begin(), m_ordered_rollouts.end(), s_static_rollouts);

    // Sort indexes by rollout cost.
    std::stable_sort(
        m_ordered_rollouts.begin(),
        m_ordered_rollouts.end(),
        [this](std::int64_t left, std::int64_t right) {
            return m_rollouts[left].cost < m_rollouts[right].cost;
        }
    );

    std::span indexes {m_ordered_rollouts};

    // Rollouts to keep.
    std::span keep = indexes.first(m_configuration.keep_best_rollouts);

    // Rollouts to resample.
    std::span resample = indexes.last(m_ordered_rollouts.size() - keep.size());

    for (std::int64_t index : keep) {
        Rollout &rollout = m_rollouts[index];

        // Shift the rollout noise to align with current time.
        rollout.noise.leftCols(m_shifted) = rollout.noise.rightCols(m_shifted).eval();

        // Add noise to the rest of the rollout.
        for (int i = m_shifted; i < m_steps; i++)
            rollout.noise.col(i) = m_gaussian();
    }

    for (std::int64_t index : resample) {
        Rollout &rollout = m_rollouts[index];

        // Add noise to the last control for the rest of the rollout.
        for (int i = 0; i < m_steps; i++)
            rollout.noise.col(i) = m_gaussian();
    }

    // The zero noise sample is always the first element, that is untouched.
    // get_rollout(0).setZero();

    // Sample negative the previous optimal noise.
    m_rollouts[1].noise = -m_gradient;
}

void Trajectory::rollout()
{
    // Get the rounded down number rollouts per thread, and the remaining
    // rollouts to distribute between the threads. The rollouts to distribute
    // will always be less than the number of threads.
    auto [each_thread, distribute] = std::div(
        m_configuration.rollouts,
        m_configuration.threads
    );

    int start = 0;
    for (unsigned int thread = 0; thread < m_configuration.threads; thread++) {
        int stop = start + each_thread;

        // If there are threads to distribute.
        if (distribute > 0) {
            stop += 1;
            distribute -= 1;
        }

        // For rollouts < threads, each_thread == 0, stop once all distributed.
        if (start == stop)
            break;

        // Rollout trajectories from [start, stop)
        auto lambda = [this, thread, start, stop]() {
            for (int i = start; i < stop; i++) {
                rollout(m_rollouts[i], *m_dynamics[thread], *m_cost[thread]);
            }
        };

        m_futures[thread] = m_thread_pool.enqueue(lambda);
        start = stop;
    }

    for (auto &future : m_futures)
        future.get();
}

void Trajectory::rollout(Rollout &rollout, Dynamics &dynamics, Cost &cost)
{
    Eigen::VectorXd state = m_rollout_state;
    dynamics.set(state);
    rollout.cost = 0.0;

    for (int step = 0; step < m_steps; ++step) {

        // Add the rollout noise to the optimal control.
        Eigen::VectorXd control = (
            m_optimal_control_shifted.col(step) +
            rollout.noise.col(step)
        );

        double step_cost = (
            std::pow(m_configuration.cost_discount_factor, step) *
            cost.get(state, control, m_configuration.time_step)
        );

        if (std::isnan(step_cost)) {
            rollout.cost = NAN;
            return;
        }

        // Cumulative running cost.
        rollout.cost += step_cost;

        // Step the dynamics simulation.
        state = dynamics.step(control, m_configuration.time_step);
    }
}

void Trajectory::optimise()
{
    double maximum = std::numeric_limits<double>::max();
    double minimum = std::numeric_limits<double>::min();

    // Get the minimum and maximum rollout cost.
    auto [it1, it2] = std::minmax_element(
        m_rollouts.begin(),
        m_rollouts.end(),
        [](auto &left, auto &right) {
            return left.cost < right.cost ? true : std::isnan(right.cost);
        }
    );

    minimum = it1->cost;
    maximum = it2->cost;

    // For parameterisation of each cost.
    double difference = maximum - minimum;
    if (difference < 1e-6)
        return;

    // Running sum of total likelihood for normalisation between zero and one.
    double total = 0.0;

    // Transform the weights to likelihoods.
    for (std::int64_t i = 0; i < m_configuration.rollouts; ++i) {
        double cost = m_rollouts[i].cost;

        // NaNs indicate a failed rollout and do not contribute anything. 
        if (std::isnan(cost)) {
            m_weights[i] = 0.0;
            continue;
        }

        double likelihood = std::exp(
            -m_configuration.cost_scale * (cost - minimum) / difference
        );

        total += likelihood;
        m_weights[i] = likelihood;
    }

    // Normalise the likelihoods.
    std::transform(
        m_weights.begin(),
        m_weights.end(),
        m_weights.begin(),
        [total](double likelihood){ return likelihood / total; }
    );

    // The optimal trajectory is a linear combination of the noise samples.
    m_gradient = m_rollouts[0].noise * m_weights[0];
    for (std::int64_t i = 1; i < m_configuration.rollouts; ++i)
        m_gradient += m_rollouts[i].noise * m_weights[i];

    if (m_configuration.filter) {
        m_filter.reset(m_rollout_time);

        for (int i = 0; i < m_steps; i++) {
            m_filter.add_measurement(
                m_gradient.col(i),
                m_rollout_time + i * m_configuration.time_step
            );
        }

        for (int i = 0; i < m_steps; i++) {
            m_filter.apply(
                m_gradient.col(i),
                m_rollout_time + i * m_configuration.time_step
            );
        }
    }

    // Step in the direction of the gradient.
    m_optimal_control_shifted += m_gradient * m_configuration.gradient_step;

    // Clip the optimal control.
    m_optimal_control = m_optimal_control
        .cwiseMin(m_configuration.control_min.replicate(1, m_steps))
        .cwiseMax(m_configuration.control_max.replicate(1, m_steps));

    // Update the optimal control under lock.
    std::scoped_lock lock(m_optimal_control_mutex);
    m_last_rollout_time = m_rollout_time;
    m_optimal_control = m_optimal_control_shifted;
}

void Trajectory::get(Eigen::Ref<Eigen::VectorXd> control, double time)
{
    assert(time >= m_last_rollout_time);

    // Steps into the current horison.
    double t = (time - m_last_rollout_time) / m_configuration.time_step;

    // Round down to integer.
    int lower = (int)t;
    int upper = lower + 1;

    std::scoped_lock lock(m_optimal_control_mutex);

    // Past the end of the trajectory. Return the specified default control
    // parameters.
    if (upper >= m_steps) {
        if (m_configuration.control_default)
            control = m_configuration.control_default.value();
        else
            control = m_optimal_control.rightCols(1);
        return;
    }

    // Steps relative to the previous step. Parameterise between lower and upper.
    t -= lower;

    // Linear interpolation between optimal controls.
    control = (
        (1.0 - t) * m_optimal_control.col(lower) +
        t * m_optimal_control.col(upper)
    );
}

} // namespace mppi
