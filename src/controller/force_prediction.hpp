#pragma once

#include <variant>
#include <shared_mutex>
#include <deque>

#include "controller/json.hpp"
#include "controller/kalman.hpp"

/**
 * @brief A predictor that observes forces and estimates future forces.
 */
class ForcePredictor
{
public:

    enum Type {
        AVERAGE,
        KALMAN
    };

    class Configuration;

    /**
     * @brief A handle to a force predictor.
     * 
     * Used to copy amongst multiple dynamics instances to allow multithreading.
     * Typically the predict() methods would hold a shared read only lock
     * with the other handles. The parent holds an exclusive write lock when
     * the force prediction update occurs.
     */
    class Handle
    {
    public:

        /**
         * @brief Predict the force at a time in the future.
         * 
         * @param time The time of the force prdiction
         * @returns The predicted force.
         */
        virtual Eigen::Vector3d predict(double time) = 0;

        /**
         * @brief Make a copy of the handle.
         */
        virtual std::unique_ptr<Handle> copy() = 0;
    };

    /**
     * @brief Create a handle to the force predictor.
     */
    virtual std::unique_ptr<Handle> create_handle() = 0;

    /**
     * @brief Create a force predictor.
     * 
     * @param configuration The configuration of the force predictor.
     */
    static std::unique_ptr<ForcePredictor> create(
        const Configuration &configuration
    );

    /**
     * @brief Update the predictor with an observed force.
     * 
     * @pre `time > previous_update_time`
     * 
     * @param force The observed force.
     * @param time The time of the observation.
     */
    virtual void update(Eigen::Vector3d force, double time) = 0;

    /**
     * @brief Update the predictor without an observed force.
     * 
     * @pre `time > previous_update_time`
     * 
     * @param time The time of the update.
     */
    virtual void update(double time) = 0;

    /**
     * @brief Predict the force at a time in the future.
     * 
     * @param time The time of the force prdiction
     * @returns The predicted force.
     */
    virtual Eigen::Vector3d predict(double time) = 0;
};

/**
 * @brief A trivial average force, that returns the observed force.
 */
class AverageForcePredictor : public ForcePredictor
{
public:

    class Configuration {

        /// The period of time over which to calculate the average force.
        double window;

        /// JSON conversion for average force predictor configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, window);
    };

    /**
     * @brief A prediction only handle to a average force predictor.
     */
    class Handle : public ForcePredictor::Handle
    {
    public:

        /**
         * @brief Yields the average force predictor force.
         * 
         * @param time The time of the force, unused.
         * @returns The average predicted force.
         */
        inline Eigen::Vector3d predict(double time) override {
            std::shared_lock lock(m_parent->m_mutex);
            return m_parent->m_average;
        };

        /**
         * @brief Make a copy of the average force predictor handle.
         * 
         * Used to copy amongst dynamics objects for multi-threading.
         */
        inline std::unique_ptr<ForcePredictor::Handle> copy() override {
            return m_parent->create_handle();
        }

    private:

        friend class AverageForcePredictor;

        /**
         * @brief Initialise a average force predictor handle.
         * @param parent The owning average force predictor.
         */
        Handle(AverageForcePredictor *parent)
            : m_parent(parent)
        {}

        /// Pointer to the average force predictor.
        std::unique_ptr<AverageForcePredictor> m_parent;
    };

    /**
     * @brief Create an average force predictor.
     * 
     * @param configuration 
     * @return std::unique_ptr<AverageForcePredictor> 
     */
    static std::unique_ptr<AverageForcePredictor> create(
        const Configuration &configuration
    );

    /**
     * @brief Create a handle to the average force predictor.
     */
    inline std::unique_ptr<ForcePredictor::Handle> create_handle() override {
        return std::unique_ptr<Handle>(new Handle(this));
    }

    /**
     * @brief Update the predictor with an observed force.
     * 
     * @param force The observed force.
     * @param time The time of the observation.
     */
    inline void update(Eigen::Vector3d force, double time) override;

    /**
     * @brief Does nothing.
     */
    inline void update(double time) override {}

    /**
     * @brief Predict the force as the last observed force, regardless of
     * time.
     * 
     * @param time The time of the force prediction, unused.
     * @returns The last observed force.
     */
    inline Eigen::Vector3d predict(double /* time */) override;

private:

    friend class Handle;

    AverageForcePredictor();

    /// Mutex protecting concurrent force read / write.
    std::shared_mutex m_mutex;

    /// The window of forces to average over. The most recent force is always
    /// kept regardless of its age.
    std::vector<std::pair<double, Eigen::Vector3d>> m_buffer;

    /// The average of the force window.
    Eigen::Vector3d m_average;
};

/**
 * @brief A force predictor based on a kalman filter.
 * 
 * Note that update() must be called every configured time_step, with or without
 * a force observation.
 */
class KalmanForcePredictor : public ForcePredictor
{
public:

    struct Configuration {

        /// The time step of the state transition matrix for the force.
        double time_step;

        /// The horison over which the force should be predicted.
        double horison;

        /// The order of the force estimation.
        unsigned int order;

        /// Set to {var(x), var(y), var(z), var(dx), var(dy), var(dz), ...}
        Eigen::VectorXd transition_variance;

        /// Set to {var(x), var(y), var(z), var(dx), var(dy), var(dz), ...}
        Eigen::VectorXd observation_variance;

        /// JSON conversion for kalman force predictor configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            time_step, horison, order, transition_variance, observation_variance
        );
    };

    /**
     * @brief A read only handle to a KalmanForcePredictor.
     */
    class Handle : public ForcePredictor::Handle
    {
    public:

        /**
         * @brief Predict the force at a time since the last kalman force
         * predictor update.
         * 
         * @param time The time of the force prediction.
         * @returns The predicted force.
         */
        inline Eigen::Vector3d predict(double time) {
            return m_parent->predict(time);
        }

        /**
         * @brief Create a copy of the kalman force predictor handle.
         */
        inline std::unique_ptr<ForcePredictor::Handle> copy() override {
            return m_parent->create_handle();
        }

    private:

        friend class KalmanForcePredictor;

        /**
         * @brief Initialise a kalman force predictor handle.
         * @param parent The kalman force predictor.
         */
        Handle(KalmanForcePredictor *parent)
            : m_parent(parent)
        {}

        /// Pointer to the kalman force predictor.
        std::shared_ptr<KalmanForcePredictor> m_parent;
    };

    /**
     * @brief Create a new kalman force predictor.
     * 
     * @param configuration The configuration of the predictor.
     * @returns A pointer to the predictor on success or nullptr on failure.
     */
    static std::unique_ptr<KalmanForcePredictor> create(
        const Configuration &configuration
    );

    /**
     * @brief Create a handle to the force predictor.
     */
    inline std::unique_ptr<ForcePredictor::Handle> create_handle() override {
        return std::unique_ptr<Handle>(new Handle(this));
    }

    /**
     * @brief Create a state transition matrix of a given order.
     * 
     * The order is the highest derivative in each observation, that is used to
     * predict the force.
     * 
     * @param time_step The time step of the transition matrix.
     * @param order The order of the state transition matrix.
     * 
     * @returns The state transition matrix.
     */
    static Eigen::MatrixXd create_euler_state_transition_matrix(
        double time_step,
        unsigned int order
    );

    /**
     * @brief Update the predictor with an observed force.
     * 
     * @param force The observed force.
     * @param time The time of the observation.
     */
    inline void update(Eigen::Vector3d force, double time) override;

    /**
     * @brief Update the predictor without a force.
     */
    inline void update(double time) override;

    /**
     * @brief Predict the force as the last observed force, regardless of
     * time.
     * 
     * @param time The time of the force prdiction
     * @returns The last observed force.
     */
    Eigen::Vector3d predict(double /* time */) override;

private:

    friend class Handle;

    KalmanForcePredictor() = default;

    /// The period of time over which the prediction is made.
    double m_horison;

    /// The expected kalman time step.
    double m_time_step;

    /// The number of steps to make in each update.
    unsigned int m_steps;

    /// The time of the last update.
    double m_last_update;

    std::shared_mutex m_mutex;

    /// The kalman filter used to estimate the current force.
    std::unique_ptr<KalmanFilter> m_kalman;

    /// The kalman filter used to estimate future forces.
    std::unique_ptr<KalmanFilter> m_predictor;

    /// The calculated state, storing force derivatives.
    Eigen::VectorXd m_state;

    /// The latest updated horison.
    Eigen::MatrixXd m_prediction;
};

/**
 * @brief Configuration of a force predictor.
 * 
 * Postpone configuration until types are fully declared.
 */
class ForcePredictor::Configuration
{
    std::variant<
        AverageForcePredictor::Configuration,
        KalmanForcePredictor::Configuration
    > config;

    inline void to_json(json &j, const ForcePredictor::Configuration &configuration)
    {
        if (std::holds_alternative<AverageForcePredictor::Configuration>(configuration.config)) {
            j["type"] = "average";
            j["configuration"] = std::get<AverageForcePredictor::Configuration>(configuration.config);
        }
        else if (std::holds_alternative<KalmanForcePredictor::Configuration>(configuration.config)) {
            j["type"] = "kalman";
            j["configuration"] = std::get<KalmanForcePredictor::Configuration>(configuration.config);
        }
    }

    inline void from_json(const json &j, ForcePredictor::Configuration &configuration)
    {
        auto type = j.at("type").get<std::string>();
        if (type == "average") {
            configuration.config = (AverageForcePredictor::Configuration)j.at("configuration");
        }
        else if (type == "kalman") {
            configuration.config = (KalmanForcePredictor::Configuration)j.at("configuration");
        }
    }
};
