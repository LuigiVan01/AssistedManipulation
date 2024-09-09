#include "mppi.hpp"

#include <cmath>
#include <numeric>
#include <span>
#include <chrono>

namespace mppi {

std::unique_ptr<Trajectory> Trajectory::create(
    const Configuration &configuration,
    std::unique_ptr<Dynamics> &&dynamics,
    std::unique_ptr<Cost> &&cost,
    std::unique_ptr<Filter> &&filter
) {
    // Ensure dynamics and cost expect the same control.
    if (dynamics->control_dof() != cost->control_dof()) {
        std::cerr << "controller dynamics control dof "
                  << dynamics->control_dof()
                  << " != cost control dof "
                  << cost->control_dof()
                  << std::endl;
        return nullptr;
    }

    // Ensure dynamics and cost expect the same state.
    if (dynamics->state_dof() != cost->state_dof()) {
        std::cerr << "controller dynamics state dof "
                  << dynamics->state_dof()
                  << " != cost state dof "
                  << cost->state_dof()
                  << std::endl;
        return nullptr;
    }

    if (configuration.control_min.size() != dynamics->control_dof() ||
        configuration.control_max.size() != dynamics->control_dof()) {
        std::cerr << "controller maximum and minimum must have length "
                  << dynamics->control_dof() << std::endl;
        return nullptr;
    }

    if (configuration.covariance.rows() != configuration.covariance.cols()) {
        std::cerr << "controller covariance matrix not square" << std::endl;
        return nullptr;
    }

    if (configuration.covariance.rows() != dynamics->control_dof()) {
        std::cerr << "controller sample variance dof " << configuration.covariance.rows()
                  << " != dynamics and cost control dof " <<  dynamics->control_dof()
                  << std::endl;
        return nullptr;
    }

    if (configuration.rollouts < 1) {
        std::cerr << "trajectory rollouts must be greater than zero" << std::endl;
        return nullptr;
    }

    if (configuration.keep_best_rollouts < 0) {
        std::cerr << "trajectory cached rollouts cannot be less than zero" << std::endl;
        return nullptr;
    }

    if (configuration.threads <= 0) {
        std::cerr << "trajectory threads must be positive nonzero" << std::endl;
        return nullptr;
    }

    return std::unique_ptr<Trajectory>(new Trajectory(
        configuration,
        std::move(dynamics),
        std::move(cost),
        std::move(filter)
    ));
}

Trajectory::Trajectory(
    const Configuration &configuration,
    std::unique_ptr<Dynamics> &&dynamics,
    std::unique_ptr<Cost> &&cost,
    std::unique_ptr<Filter> &&filter
) noexcept
  : m_step_count(std::ceil(configuration.horison / configuration.time_step))
  , m_time_step(configuration.time_step)
  , m_rollout_count(configuration.rollouts + s_static_rollouts)
  , m_thread_count(configuration.threads)
  , m_state_dof(dynamics->state_dof())
  , m_control_dof(dynamics->control_dof())
  , m_update_last(0)
  , m_update_duration(0)
  , m_update_count(0)
  , m_thread_pool(configuration.threads)
  , m_dynamics(configuration.threads)
  , m_cost(configuration.threads)
  , m_futures(configuration.threads)
  , m_gaussian(configuration.covariance)
  , m_rollout_state(configuration.initial_state)
  , m_rollout_time(0.0)
  , m_last_rollout_time(0.0)
  , m_cost_discount_factor(configuration.cost_discount_factor)
  , m_cost_scale(configuration.cost_scale)
  , m_shift_by(0)
  , m_shifted(0)
  , m_rollouts(m_rollout_count, Rollout(dynamics->control_dof(), m_step_count))
  , m_weights(m_rollout_count, 1) // (rows, cols) ...
  , m_gradient(dynamics->control_dof(), m_step_count)
  , m_gradient_step(configuration.gradient_step)
  , m_filter(std::move(filter))
  , m_optimal_control_shifted(dynamics->control_dof(), m_step_count)
  , m_optimal_rollout(dynamics->control_dof(), m_step_count)
  , m_optimal_control(dynamics->control_dof(), m_step_count)
  , m_keep_best_rollouts(configuration.keep_best_rollouts)
  , m_ordered_rollouts(configuration.rollouts)
  , m_bound_control(configuration.control_bound)
  , m_control_min(configuration.control_min)
  , m_control_max(configuration.control_max)
  , m_control_default(configuration.control_default)
{
    m_weights.setZero();
    m_gradient.setZero();
    m_optimal_control.setZero();

    // Set the initial gaussian noise to zero.
    for (auto &rollout : m_rollouts) {
        rollout.noise.setZero();
    }

    // Ensure the passed dynamics and cost are not destroyed.
    m_dynamics[0] = std::move(dynamics);
    m_cost[0] = std::move(cost);

    // Initialise thread data.
    for (unsigned int i = 1; i < configuration.threads; i++) {
        m_dynamics[i] = m_dynamics[0]->copy();
        m_cost[i] = m_cost[0]->copy();
    }

    if (configuration.smoothing) {
        m_smoothing_filter = SavitzkyGolayFilter(
            m_step_count,
            m_control_dof,
            configuration.smoothing->window,
            configuration.smoothing->order,
            0,
            configuration.time_step
        );
    }
}

void Trajectory::update(const Eigen::Ref<Eigen::VectorXd> state, double time)
{
    using namespace std::chrono;

    m_rollout_state = state;
    m_rollout_time = time;

    auto start = steady_clock::now();

    // Sample all the control trajectories for each rollout.
    sample(time);

    // Calculate the cost of each sampled rollout.
    rollout();

    // Take a fancy linear combination of the rollouts to generate a gradient
    // to step the final control trajectory towards.
    optimise();

    // Rollout the updated optimal trajectory, and apply the optional filter at
    // each time step. The rollout also computes optimal trajectory cost.
    filter();

    // Update the optimal control trajectory under lock.
    {
        std::scoped_lock lock(m_optimal_control_mutex);
        m_last_rollout_time = m_rollout_time;
        m_optimal_control = m_optimal_control_shifted;
    }

    m_update_duration = duration<double>(steady_clock::now() - start).count();
    m_update_last = time;
    ++m_update_count;
}

void Trajectory::sample(double time)
{
    // The number of time steps to shift the optimal control and best sampled
    // trajectories forward by, to align them with the current time.
    m_shift_by = (std::int64_t)((time - m_last_rollout_time) / m_time_step);

    // The number of columns to shift back.
    m_shifted = m_step_count - m_shift_by;

    // Shift the the optimal control to align with the current time.
    m_optimal_control_shifted.leftCols(m_shifted) = m_optimal_control.rightCols(m_shifted).eval();
    m_optimal_control_shifted.rightCols(m_shift_by) = m_optimal_control.rightCols(1).replicate(1, m_shift_by);

    // Reset to random noise if all trajectories are out of date.
    if (m_shift_by >= m_step_count) {
        for (std::int64_t index = s_static_rollouts; index < m_rollout_count; ++index) {
            Rollout &rollout = m_rollouts[index];
            for (int i = 0; i < m_step_count; i++)
                rollout.noise.col(i) = m_gaussian();
        }
        return;
    }

    // Regenerate indexes of rollouts to sort by cost as 2, 2 + 1, 2 + 2, ...
    // Where index 0 is zero sampled noise and index 1 is the negative gradient
    // that are always kept.
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
    std::span keep = indexes.first(m_keep_best_rollouts);

    // Rollouts to resample.
    std::span resample = indexes.last(m_ordered_rollouts.size() - keep.size());

    for (std::int64_t index : keep) {
        Rollout &rollout = m_rollouts[index];

        // Shift the rollout noise to align with current time.
        rollout.noise.leftCols(m_shifted) = rollout.noise.rightCols(m_shifted).eval();

        // Add noise to the rest of the rollout.
        for (int i = m_shifted; i < m_step_count; i++)
            rollout.noise.col(i) = m_gaussian();
    }

    for (std::int64_t index : resample) {
        Rollout &rollout = m_rollouts[index];

        // Add noise to the last control for the rest of the rollout.
        for (int i = 0; i < m_step_count; i++) {
            rollout.noise.col(i) = m_gaussian();
        }
    }

    // The zero noise sample is always the first element, that is untouched.
    // get_rollout(0).setZero();

    // Sample negative the previous optimal noise.
    m_rollouts[1].noise = -m_optimal_control;
}

void Trajectory::rollout()
{
    // Get the rounded down number rollouts per thread, and the remaining
    // rollouts to distribute between the threads. The rollouts to distribute
    // will always be less than the number of threads.
    auto [each_thread, distribute] = std::div(m_rollout_count, m_thread_count);

    int start = 0;
    for (unsigned int thread = 0; thread < m_thread_count; thread++) {
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
                rollout(&m_rollouts[i], m_dynamics[thread].get(), m_cost[thread].get());
            }
        };

        m_futures[thread] = m_thread_pool.enqueue(lambda);
        start = stop;
    }

    // Barrier waiting for all threads to complete.
    for (auto &future : m_futures)
        future.get();
}

void Trajectory::rollout(Rollout *rollout, Dynamics *dynamics, Cost *cost)
{
    Eigen::VectorXd state = m_rollout_state;
    dynamics->set(state, m_rollout_time);
    cost->reset();
    rollout->cost = 0.0;

    for (int step = 0; step < m_step_count; ++step) {

        // Add the rollout noise to the optimal control.
        Eigen::VectorXd control = (
            m_optimal_control_shifted.col(step) +
            rollout->noise.col(step)
        );

        double step_cost = (
            std::pow(m_cost_discount_factor, step) *
            cost->get(state, control, dynamics, m_rollout_time + step * m_time_step)
        );

        // Rollout weight is interpreted as zero during optimisation.
        if (std::isnan(step_cost)) {
            rollout->cost = NAN;
            return;
        }

        // Cumulative running cost.
        rollout->cost += step_cost;

        // Step the dynamics simulation.
        state = dynamics->step(control, m_time_step);
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
    for (std::int64_t i = 0; i < m_rollout_count; ++i) {
        double cost = m_rollouts[i].cost;

        // NaNs indicate a failed rollout and do not contribute anything. 
        if (std::isnan(cost)) {
            m_weights[i] = 0.0;
            continue;
        }

        double likelihood = std::exp(
            -m_cost_scale * (cost - minimum) / difference
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
    for (std::int64_t i = 1; i < m_rollout_count; ++i) {
        m_gradient += m_rollouts[i].noise * m_weights[i];
    }

    // Step in the direction of the gradient.
    m_optimal_control_shifted += m_gradient * m_gradient_step;

    // Smoothing filter for rollouts.
    if (m_smoothing_filter) {
        m_smoothing_filter->reset(m_rollout_time);

        for (int i = 0; i < m_step_count; i++) {
            m_smoothing_filter->add_measurement(
                m_optimal_control_shifted.col(i),
                m_rollout_time + i * m_time_step
            );
        }

        for (int i = 0; i < m_step_count; i++) {
            m_smoothing_filter->apply(
                m_optimal_control_shifted.col(i),
                m_rollout_time + i * m_time_step
            );
        }
    }

    // Clip the optimal control.
    if (m_bound_control) {
        m_optimal_control_shifted = m_optimal_control_shifted
            .cwiseMin(m_control_max.replicate(1, m_step_count))
            .cwiseMax(m_control_min.replicate(1, m_step_count));
    }
}

void Trajectory::filter()
{
    Eigen::VectorXd state = m_rollout_state;
    Dynamics *dynamics = m_dynamics[0].get();
    Cost *cost = m_cost[0].get();

    dynamics->set(state, m_rollout_time);
    cost->reset();
    m_optimal_rollout.cost = 0.0;

    for (int step = 0; step < m_step_count; ++step) {

        auto control = m_optimal_control_shifted.col(step);

        // Apply the filter to the optimal control.
        if (m_filter)
            m_filter->filter(state, control, m_rollout_time + step * m_time_step);

        double step_cost = (
            std::pow(m_cost_discount_factor, step) *
            cost->get(state, control, dynamics, m_rollout_time + step * m_time_step)
        );

        assert(!std::isnan(step_cost));

        // Cumulative running cost.
        m_optimal_rollout.cost += step_cost;

        // Step the dynamics simulation.
        state = dynamics->step(control, m_time_step);
    }
}

void Trajectory::get(Eigen::Ref<Eigen::VectorXd> control, double time)
{
    assert(time >= m_last_rollout_time);

    // Steps into the current horison.
    double t = (time - m_last_rollout_time) / m_time_step;

    // Round down to integer.
    int lower = (int)t;
    int upper = lower + 1;

    std::scoped_lock lock(m_optimal_control_mutex);

    // Past the end of the trajectory. Return the specified default control
    // parameters.
    if (upper >= m_step_count) {
        if (m_control_default)
            control = m_control_default.value();
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
