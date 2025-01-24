#pragma once

#include <cstdlib>
#include <filesystem>

#include "simulation/simulator.hpp"
#include "simulation/frankaridgeback/actor.hpp"
#include "frankaridgeback/objective/assisted_manipulation.hpp"
#include "frankaridgeback/objective/track_point.hpp"
#include "logging/mppi.hpp"
#include "logging/frankaridgeback.hpp"
#include "logging/assisted_manipulation.hpp"
#include "test/test.hpp"

class BaseTest : public RegisteredTest<BaseTest>
{
public:

    static inline constexpr const char *TEST_NAME = "base";

    
    static inline bool registration = []() {
        std::cout << "Base test header included" << std::endl;
        return register_test();
    }();

    struct Configuration {

        /// The output folder for the test.
        std::filesystem::path folder;

        /// Duration of the test.
        double duration;

        /// Simulation configuration.
        Simulator::Configuration simulator;

        /// The actors configuration including controller update rate.
        FrankaRidgeback::Actor::Configuration actor;

        /// MPPI logging configuration.
        logger::MPPI::Configuration mppi_logger;

        /// Frankaridgeback logging configuration.
        logger::FrankaRidgebackDynamics::Configuration dynamics_logger;

        /// Dynamics forecast logging configuration.
        logger::FrankaRidgebackDynamicsForecast::Configuration forecast_logger;

        /// Objective function logging configuration.
        logger::AssistedManipulation::Configuration objective_logger;

        // JSON conversion for reach for point test configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, duration, simulator, actor, mppi_logger, dynamics_logger,
            forecast_logger, objective_logger
        )
    };

    static inline const Configuration DEFAULT_CONFIGURATION = {
        .folder = "default",
        .duration = 15.0,
        .simulator = {
            .time_step = 0.005,
            .gravity = {0.0, 0.0, 9.81}
        },
        .actor = {
            .mppi = {
                .configuration = {
                    .initial_state = make_state(FrankaRidgeback::Preset::HUDDLED),
                    .rollouts = 50,
                    .keep_best_rollouts = 20,
                    .time_step = 0.01,
                    .horison = 0.3,
                    .gradient_step = 2.0,
                    .cost_scale = 10.0,
                    .cost_discount_factor = 1.0,
                    .covariance = FrankaRidgeback::Control{
                        0.1, 0.1, 0.2, // base
                        7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, // arm
                        0.0, 0.0 // gripper
                    }.asDiagonal(),
                    .control_bound = true,
                    .control_min = FrankaRidgeback::Control{
                        -0.5, -0.5, -1.0, // base
                        -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, //arm
                        -0.05, -0.05 // gripper
                    },
                    .control_max = FrankaRidgeback::Control{
                        0.5, 0.5, 1.0, // base
                        100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, // arm
                        0.05, 0.05 // gripper
                    },
                    .control_default = FrankaRidgeback::Control::Zero(),
                    .smoothing = mppi::Configuration::Smoothing {
                        .window = 10,
                        .order = 1
                    },
                    .threads = 12
                },
                .dynamics = {
                    .type = FrankaRidgeback::SimulatorDynamics::Configuration::Type::RAISIM,
                    .raisim = FrankaRidgeback::RaisimDynamics::DEFAULT_CONFIGURATION,
                    .pinocchio = FrankaRidgeback::PinocchioDynamics::DEFAULT_CONFIGURATION
                }
            },
            .dynamics = {
                .type = FrankaRidgeback::SimulatorDynamics::Configuration::Type::RAISIM,
                .raisim = FrankaRidgeback::RaisimDynamics::DEFAULT_CONFIGURATION,
                .pinocchio = FrankaRidgeback::PinocchioDynamics::DEFAULT_CONFIGURATION
            },
            .objective = {
                .type = FrankaRidgeback::Actor::Configuration::Objective::Type::ASSISTED_MANIPULATION,
                .assisted_manipulation = FrankaRidgeback::AssistedManipulation::DEFAULT_CONFIGURATION,
                .track_point = FrankaRidgeback::TrackPoint::DEFAULT_CONFIGURATION
            },
            .forecast = FrankaRidgeback::Actor::Configuration::Forecast {
                .configuration = {
                    .time_step = 0.01,
                    .horison = 0.3,
                    .end_effector_wrench_forecast = {
                        .type = Forecast::Configuration::Type::AVERAGE,
                        .locf = LOCFForecast::Configuration {
                            .observation = Vector6d::Zero(),
                            .horison = 0.3
                        },
                        .average = AverageForecast::Configuration {
                            .states = 6,
                            .window = 0.3
                        },
                        .kalman = KalmanForecast::Configuration {
                            .observed_states = 6,
                            .time_step = 0.01,
                            .horison = 0.3,
                            .order = 1,
                            .variance = Vector6d::Ones() * 0.2,
                            .initial_state = Vector6d::Zero()
                        }
                    }
                },
                .dynamics = {
                    .type = FrankaRidgeback::SimulatorDynamics::Configuration::Type::RAISIM,
                    .raisim = FrankaRidgeback::RaisimDynamics::DEFAULT_CONFIGURATION,
                    .pinocchio = FrankaRidgeback::PinocchioDynamics::DEFAULT_CONFIGURATION
                }
            },
            .controller_rate = 0.05,
            .controller_substeps = 1,
            .forecast_rate = 0.00
        },
        .mppi_logger = {
            .folder = "",
            .state_dof = FrankaRidgeback::DoF::STATE,
            .control_dof = FrankaRidgeback::DoF::CONTROL,
            .rollouts = 0,
            .log_costs = true,
            .log_weights = true,
            .log_gradient = true,
            .log_optimal_rollout = true,
            .log_optimal_cost = true,
            .log_update = true
        },
        .dynamics_logger = {
            .folder = "",
            .log_joints = true,
            .log_control = true,
            .log_end_effector_position = true,
            .log_end_effector_orientation = false,
            .log_end_effector_velocity = true,
            .log_end_effector_acceleration = true,
            .log_power = true,
            .log_tank_energy = true,
        },
        .forecast_logger = {
            .folder = "",
            .log_end_effector_position = true,
            .log_end_effector_orientation = false,
            .log_end_effector_velocity = true,
            .log_end_effector_acceleration = true,
            .log_power = true,
            .log_tank_energy = true,
            .log_wrench = true
        },
        .objective_logger = {
            .folder = "",
            .log_joint_limit = true,
            .log_self_collision_limit = true,
            .log_workspace_limit = true,
            .log_energy_limit = true,
            .log_velocity_cost = true,
            .log_trajectory_cost = true,
            .log_manipulability_cost = true,
            .log_total = true
        }
    };

    /**
     * @brief Create a test reaching for a point.
     * 
     * @param options The test options. The configuration overrides from the
     * default configuration.
     * 
     * @return A pointer to the test on success or nullptr on failure.
     */
    static std::unique_ptr<BaseTest> create(Options &options);

    /**
     * @brief Create a the base simulation.
     * 
     * @param configuration The configuration of the base simulation.
     * @returns A pointer to the bast simulation on success or nullptr on failure.
     */
    static std::unique_ptr<BaseTest> create(const Configuration &configuration);

    /**
     * @brief Get the simulator.
     */
    inline Simulator *get_simulator()
    {
        return m_simulator.get();
    }

    /**
     * @brief Get the franka-ridgeback actor.
     */
    inline FrankaRidgeback::Actor *get_frankaridgeback()
    {
        return m_frankaridgeback.get();
    }

    /**
     * @brief Step the simulation.
     * 
     * Can be called by other test cases to step the simulation, without having
     * to handle mppi logging.
     */
    void step();

    /**
     * @brief Run the test.
     * @returns If the test was successful.
     */
    bool run() override;

private:

    /**
     * @brief Initialise the base simulation.
     * 
     * @param configuration The configuration of the simulation.
     * @param simulator The simulator.
     * @param frankaridgeback The frankaridgeback instance being simulated.
     * @param mppi_logger Logger for the mppi.
     * @param dynamics_logger Logger for the simulated actor dynamics.
     * @param forecast_logger Logger for the forecast dynamics.
     */
    BaseTest(
        double duration,
        std::unique_ptr<Simulator> &&simulator,
        std::shared_ptr<FrankaRidgeback::Actor> &&frankaridgeback,
        std::unique_ptr<logger::MPPI> &&mppi_logger,
        std::unique_ptr<logger::FrankaRidgebackDynamics> &&dynamics_logger,
        std::unique_ptr<logger::FrankaRidgebackDynamicsForecast> &&forecast_logger,
        std::unique_ptr<logger::AssistedManipulation> &&objective_logger
    );

    /// Duration of the test when run.
    double m_duration;

    /// Pointer to the simulator.
    std::unique_ptr<Simulator> m_simulator;

    /// Pointer to the franka ridgeback being simulated.
    std::shared_ptr<FrankaRidgeback::Actor> m_frankaridgeback;

    /// Logger for the frankaridgeback mppi trajectory generator.
    std::unique_ptr<logger::MPPI> m_mppi_logger;

    /// Logger for the frankaridgeback.
    std::unique_ptr<logger::FrankaRidgebackDynamics> m_dynamics_logger;

    /// Logger for the forecast.
    std::unique_ptr<logger::FrankaRidgebackDynamicsForecast> m_forecast_logger;

    /// Logger for the objective.
    std::unique_ptr<logger::AssistedManipulation> m_objective_logger;
};
