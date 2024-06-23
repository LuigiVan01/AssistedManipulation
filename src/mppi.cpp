#include "mppi.hpp"

namespace mppi {

std::unique_ptr<Trajectory> Trajectory::create(
    Dynamics *dynamics,
    Cost *cost,
    const Configuration &configuration,
    const Eigen::VectorXd &state
) {
    // TODO: Validation goes here.
    if (configuration.rollouts < 1 || configuration.rollouts_cached < 0)
        return nullptr;

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
  , m_current_state(state)
  , m_times(steps, 1) // (rows, cols) ...
  , m_controls(configuration.rollouts * dynamics->control_dof(), steps)
  , m_costs(configuration.rollouts, 1)
  , m_weights(configuration.rollouts, 1)
  , m_optimal_control(dynamics->control_dof(), steps)
  , m_gradient(dynamics->control_dof(), steps)
{
    m_optimal_control.setZero();

    m_times.setZero();
    m_controls.setZero();
    m_costs.setZero();
    m_weights.setZero();
    m_gradient.setZero();
}

void Trajectory::update(const Eigen::VectorXd &state, double time)
{
    m_current_state = state;

    // Calculate the timestamps at each time increment.
    for (int i = 0; i < m_steps; ++i)
        m_times[i] = time + i * m_configuration.step_size;

    // Sample control trajectories. 
    sample();

    // std::cout << m_controls << std::endl;

    // Rollout
    for (std::int64_t i = 0; i < m_configuration.rollouts; i++)
        rollout(i);

    optimise();
}

void Trajectory::sample()
{
    // TODO: Warmstart sampling, zero velocity sample, negative previous
    // optimal trajectory.

    // Set all trajectories to random noise.
    m_controls = m_controls.unaryExpr(
        [&](float){ return m_distribution(m_generator); }
    );

    // Relative to the previous optimal trajectory.
    m_controls += m_optimal_control.replicate(m_configuration.rollouts, 1);
}

void Trajectory::rollout(std::int64_t i)
{
    Eigen::VectorXd state = m_current_state;

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

        // m_states.block(i * m_state_dof, step, m_state_dof, 1) = state;
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
    m_gradient.setZero();
    for (std::int64_t rollout = 0; rollout < m_configuration.rollouts; ++rollout)
        m_gradient += controls(rollout) * m_weights[rollout];

    // std::cout << "gradient: " << m_gradient.transpose() << std::endl;

    // Clip gradient.
    m_gradient = m_gradient
        .cwiseMax(-m_configuration.gradient_minmax)
        .cwiseMin(m_configuration.gradient_minmax);

    // std::cout << "gradient_clipped: " << m_gradient.transpose() << std::endl;

    m_optimal_control += m_gradient * m_configuration.gradient_step;

    // std::cout << "optimal: " << m_optimal_control << std::endl;
}

Eigen::VectorXd Trajectory::get(double time) const
{
    auto it = std::upper_bound(m_times.begin(), m_times.end(), time);
    auto index = std::distance(m_times.begin(), it);

    // Past the end of the trajectory. Return the specified default control
    // parameters.
    if (it == m_times.end()) {
        if (m_configuration.control_default_last) {
            return m_optimal_control.rightCols(1);
        }
        return m_configuration.control_default_value;
    }

    assert(m_times.rows() > 1);
    auto previous = it - 1;

    // Parameterisation of the time between nearest two timestamps.
    double t = (time - *previous) / (*it - *previous);

    // Linear interpolation between optimal controls.
    return (
        (1.0 - t) * m_optimal_control.col(index - 1) +
        t * m_optimal_control.col(index)
    );
}

} // namespace mppi
