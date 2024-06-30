#include "mppi.hpp"

#include <cmath>
#include <numeric>
#include <span>

namespace mppi {

std::mt19937 Gaussian::s_generator {};
std::normal_distribution<double> Gaussian::s_distribution {};

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

    // Count the zero and negative sample in the number of rollouts.
    configured.rollouts += s_static_rollouts;

    // Number of time steps per rollout.
    int steps = std::ceil(configured.horison / configured.step_size);

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
  , m_dynamics(dynamics)
  , m_cost(cost)
  , m_gaussian(configuration.covariance)
  , m_steps(steps)
  , m_state_dof(dynamics->state_dof())
  , m_control_dof(dynamics->control_dof())
  , m_rollout_state(state)
  , m_last_rollout_time(0.0)
  , m_rollouts(configuration.rollouts * dynamics->control_dof(), steps) // (rows, cols) ...
  , m_costs(configuration.rollouts, 1)
  , m_weights(configuration.rollouts, 1)
  , m_gradient(dynamics->control_dof(), steps)
  , m_optimal_control_shifted(dynamics->control_dof(), steps)
  , m_optimal_control(dynamics->control_dof(), steps)
  , m_ordered_rollouts(configuration.rollouts - s_static_rollouts)
{
    m_optimal_control.setZero();
    m_rollouts.setZero();
    m_costs.setZero();
    m_weights.setZero();
    m_gradient.setZero();
}

void Trajectory::update(const Eigen::Ref<Eigen::VectorXd> state, double time)
{
    m_rollout_state = state;
    m_rollout_time = time;

    sample(time);

    for (std::int64_t i = 0; i < m_configuration.rollouts; i++)
        rollout(i);

    optimise();
}

void Trajectory::sample(double time)
{
    // The number of time steps to shift the optimal control and best sampled
    // trajectories forward by, to align them with the current time.
    m_shift_by = (std::int64_t)((time - m_last_rollout_time) / m_configuration.step_size);

    // The number of columns to shift back.
    m_shifted = m_steps - m_shift_by;

    // Shift the the optimal control to align with the current time.
    m_optimal_control_shifted.leftCols(m_shifted) = m_optimal_control.rightCols(m_shifted).eval();
    m_optimal_control_shifted.rightCols(m_shift_by) = m_optimal_control.rightCols(1).replicate(1, m_shift_by);

    // Reset to random noise if all trajectories are out of date.
    if (m_shift_by >= m_steps) {
        for (std::int64_t index = s_static_rollouts; index < m_configuration.rollouts; ++index) {
            auto rollout = get_rollout(index);
            for (int i = 0; i < m_steps; i++)
                rollout.col(i) = m_gaussian();
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
            return m_costs[left] < m_costs[right];
        }
    );

    std::span indexes {m_ordered_rollouts};

    // Rollouts to keep.
    std::span keep = indexes.first(m_configuration.keep_best_rollouts);

    for (std::int64_t index : keep) {
        auto rollout = get_rollout(index);

        // Shift the rollout noise to align with current time.
        rollout.leftCols(m_shifted) = rollout.rightCols(m_shifted).eval();

        // Add noise to the rest of the rollout.
        for (int i = m_shifted; i < m_steps; i++)
            rollout.col(i) = m_gaussian();
    }

    // Rollouts to resample.
    std::span resample = indexes.last(m_ordered_rollouts.size() - keep.size());

    for (std::int64_t index : resample) {
        auto rollout = get_rollout(index);

        // Add noise to the last control for the rest of the rollout.
        for (int i = 0; i < m_steps; i++)
            rollout.col(i) = m_gaussian();
    }

    // The zero noise sample is always the first element, that is untouched.
    // get_rollout(0).setZero();

    // Sample negative the previous optimal noise.
    get_rollout(1) = -m_gradient;
}

void Trajectory::rollout(std::int64_t i)
{
    Eigen::VectorXd state = m_rollout_state;
    auto rollout = get_rollout(i);

    m_dynamics->set(state);
    m_costs[i] = 0.0;

    std::cout << "rollout " << i << std::endl;
    for (int step = 0; step < m_steps; ++step) {

        // Add the rollout noise to the optimal control.
        Eigen::VectorXd control = m_optimal_control_shifted.col(step) + rollout.col(step);

        double cost = (
            std::pow(m_configuration.cost_discount_factor, step) *
            m_cost->get(state, control, m_configuration.step_size)
        );

        if (std::isnan(cost)) {
            m_costs[m_steps] = NAN;
            return;
        }

        // Cumulative running cost.
        m_costs[i] += cost;

        // Step the dynamics simulation.
        state = m_dynamics->step(control, m_configuration.step_size);
        std::cout << state[0] << std::endl;
    }
}

void Trajectory::optimise()
{
    double maximum = std::numeric_limits<double>::max();
    double minimum = std::numeric_limits<double>::min();

    // Get the minimum and maximum rollout cost.
    auto [it1, it2] = std::minmax_element(
        m_costs.begin(),
        m_costs.end(),
        [](double a, double b) { return a < b ? true : std::isnan(b); }
    );

    minimum = *it1;
    maximum = *it2;

    std::cout << minimum << std::endl;

    // For parameterisation of each cost.
    double difference = maximum - minimum;
    if (difference < 1e-6)
        return;

    // Running sum of total likelihood for normalisation between zero and one.
    double total = 0.0;

    // Transform the weights to likelihoods.
    for (std::int64_t rollout = 0; rollout < m_configuration.rollouts; ++rollout) {
        double cost = m_costs[rollout];

        // NaNs indicate a failed rollout and do not contribute anything. 
        if (std::isnan(cost)) {
            m_weights[rollout] = 0.0;
            continue;
        }

        double likelihood = std::exp(
            -m_configuration.cost_scale * (cost - minimum) / difference
        );

        total += likelihood;
        m_weights[rollout] = likelihood;
    }

    // Normalise the likelihoods.
    std::transform(
        m_weights.begin(),
        m_weights.end(),
        m_weights.begin(),
        [total](double likelihood){ return likelihood / total; }
    );

    // The optimal trajectory is the weighted samples.
    m_gradient = get_rollout(0) * m_weights[0];
    for (std::int64_t rollout = 1; rollout < m_configuration.rollouts; ++rollout)
        m_gradient += get_rollout(rollout) * m_weights[rollout];

    // Clip gradient.
    m_gradient = m_gradient
        .cwiseMax(-m_configuration.gradient_minmax)
        .cwiseMin(m_configuration.gradient_minmax);

    // Update the optimal control under lock.
    std::scoped_lock lock(m_optimal_control_mutex);
    m_last_rollout_time = m_rollout_time;
    m_optimal_control = m_optimal_control_shifted + m_gradient * m_configuration.gradient_step;
}

void Trajectory::get(Eigen::Ref<Eigen::VectorXd> control, double time)
{
    assert(time >= m_last_rollout_time);

    // Steps into the current horison.
    double t = (time - m_last_rollout_time) / m_configuration.step_size;

    // Round down to integer.
    int lower = (int)t;
    int upper = lower + 1;

    std::scoped_lock lock(m_optimal_control_mutex);

    // Past the end of the trajectory. Return the specified default control
    // parameters.
    if (upper >= m_steps) {
        if (m_configuration.control_default_last)
            control = m_optimal_control.rightCols(1);
        else
            control = m_configuration.control_default_value;
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
