#include "mppi.hpp"

#include <cmath>

namespace mppi {

std::unique_ptr<Trajectory> Trajectory::create(
    Dynamics *dynamics,
    Cost *cost,
    const Configuration &configuration,
    const Eigen::VectorXd &state
) {
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

    if (configuration.rollouts < 1) {
        std::cerr << "trajectory rollouts must be greater than zero" << std::endl;
        return nullptr;
    }

    if (configuration.keep_best_rollouts < 0) {
        std::cerr << "trajectory cached rollouts cannot be less than zero" << std::endl;
        return nullptr;
    }

    // Number of time steps per rollout.
    int steps = std::ceil(configuration.horison / configuration.step_size);

    return std::unique_ptr<Trajectory>(
        new Trajectory(
            dynamics,
            cost,
            configuration,
            state,
            steps
        )
    );
}

Trajectory::Trajectory(
    Dynamics *dynamics,
    Cost *cost,
    const Configuration &configuration,
    const Eigen::VectorXd &state,
    int steps
) noexcept
  : m_configuration(configuration)
  , m_dynamics(dynamics)
  , m_cost(cost)
  , m_generator(std::random_device{}())
  , m_distribution(0.0, 0.1)
  , m_steps(steps)
  , m_state_dof(dynamics->state_dof())
  , m_control_dof(dynamics->control_dof())
  , m_rollout_state(state)
  , m_controls(configuration.rollouts * dynamics->control_dof(), steps) // (rows, cols) ...
  , m_costs(configuration.rollouts, 1)
  , m_weights(configuration.rollouts, 1)
  , m_gradient(dynamics->control_dof(), steps)
  , m_optimal_control(dynamics->control_dof(), steps)
{
    m_optimal_control.setZero();
    m_controls.setZero();
    m_costs.setZero();
    m_weights.setZero();
    m_gradient.setZero();
}

void Trajectory::update(const Eigen::VectorXd &state, double time)
{
    m_rollout_state = state;

    // Sample control trajectories. 
    sample();

    // Rollout
    for (std::int64_t i = 0; i < m_configuration.rollouts; i++)
        rollout(i);

    // Optimise for the gradient step.
    optimise();

    std::scoped_lock<std::mutex> lock(m_double_buffer_mutex);

    // Update optimal rollout.
    m_rollout_time = time;
    m_optimal_control += m_gradient * m_configuration.gradient_step;

    // std::cout << "optimal: " << m_optimal_control << std::endl;
}

void Trajectory::sample()
{
    // TODO: Warmstart sampling, zero velocity sample, negative previous
    // optimal trajectory.

    // Warmstart. Find and keep the best rollouts.
    // std::vector<int> best_indicies(m_configuration.keep_best_rollouts);
    // std::vector<double> best_costs(m_configuration.keep_best_rollouts, std::numeric_limits<double>::max());

    // for (int i = 0; i < m_configuration.rollouts; ++i) {
    //     auto it = std::lower_bound(best_costs.begin(), best_costs.end(), m_costs[i]);
    //     if ()
    //     best_indicies[std::distance(best_costs.begin(), it)] = i;
    // }

    // Set all trajectories to random noise.
    m_controls = m_controls.unaryExpr(
        [&](float){ return m_distribution(m_generator); }
    );

    // Add the mean (the previous optimal trajectory) to the samples.
    m_controls += m_optimal_control.replicate(m_configuration.rollouts, 1);

    // std::cout << m_controls << std::endl;
}

void Trajectory::rollout(std::int64_t i)
{
    Eigen::VectorXd state = m_rollout_state;

    m_dynamics->set(state);
    m_costs[i] = 0.0;

    for (int step = 0; step < m_steps; ++step) {
        auto control = m_controls.block(i * m_control_dof, step, m_control_dof, 1);

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

        // Step the dynamics simulation and store.
        state = m_dynamics->step(control, m_configuration.step_size);
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

    // std::cout << "costs: " << m_costs.transpose() << std::endl;
    // std::cout << "weights: " << m_weights.transpose() << std::endl;

    // Normalise the likelihoods.
    std::transform(
        m_weights.begin(),
        m_weights.end(),
        m_weights.begin(),
        [total](double likelihood){ return likelihood / total; }
    );

    // std::cout << "normalised: " << m_weights.transpose() << std::endl;

    // The optimal trajectory is the weighted samples.
    m_gradient = controls(0) * m_weights[0];
    for (std::int64_t rollout = 1; rollout < m_configuration.rollouts; ++rollout)
        m_gradient += controls(rollout) * m_weights[rollout];

    // std::cout << "gradient: " << m_gradient.transpose() << std::endl;

    // Clip gradient.
    m_gradient = m_gradient
        .cwiseMax(-m_configuration.gradient_minmax)
        .cwiseMin(m_configuration.gradient_minmax);

    // std::cout << "gradient_clipped: " << m_gradient.transpose() << std::endl;
}

void Trajectory::get(Eigen::VectorXd &control, double time)
{
    assert(time <= m_rollout_time);

    // The time relative to the last optimal trajectory generation.
    double current = time - m_rollout_time;

    // Round down to integer.
    int lower = (int)(current / m_configuration.step_size);
    int upper = lower + 1;

    std::scoped_lock<std::mutex> lock(m_double_buffer_mutex);

    // Past the end of the trajectory. Return the specified default control
    // parameters.
    if (upper >= m_steps) {
        if (m_configuration.control_default_last)
            control = m_optimal_control.rightCols(1);
        else
            control = m_configuration.control_default_value;
        return;
    }

    // Parameterise current time between lower and upper.
    double t = current - lower * m_configuration.step_size;

    // Linear interpolation between optimal controls.
    control = (
        (1.0 - t) * m_optimal_control.col(lower) +
        t * m_optimal_control.col(upper)
    );
}

} // namespace mppi
