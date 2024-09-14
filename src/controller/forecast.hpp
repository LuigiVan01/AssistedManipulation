#pragma once

#include <variant>
#include <shared_mutex>
#include <deque>

#include "controller/json.hpp"
#include "controller/kalman.hpp"

/**
 * @brief A predictor that observes forces and estimates future forces.
 */
class Forecast
{
public:

    enum Type {
        AVERAGE,
        KALMAN
    };

    struct Configuration;

    /**
     * @brief A handle to a forecast.
     * 
     * Used to copy amongst multiple dynamics instances to allow multithreading.
     * Typically the forecast() methods would hold a shared read only lock
     * with the other handles. The parent holds an exclusive write lock when
     * the forecast prediction update occurs.
     */
    class Handle
    {
    public:

        /**
         * @brief Forecast at a time in the future.
         * 
         * @param time The time of the forecast.
         * @returns The predicted state.
         */
        virtual Eigen::VectorXd forecast(double time) = 0;

        /**
         * @brief Make a copy of the handle.
         */
        virtual std::unique_ptr<Handle> copy() = 0;
    };

    /**
     * @brief Create a handle to the forecast.
     */
    virtual std::unique_ptr<Handle> create_handle() = 0;

    /**
     * @brief Create a forecast.
     * 
     * @param configuration The configuration of the forecast.
     */
    static std::unique_ptr<Forecast> create(
        const Configuration &configuration
    );

    /**
     * @brief Update the forecast with an observed measurement.
     * 
     * @pre `time > previous_update_time`
     * 
     * @param measurement The observed measurement.
     * @param time The time of the observation.
     */
    virtual void update(Eigen::VectorXd measurement, double time) = 0;

    /**
     * @brief Update the forecast without an observed measurement.
     * 
     * @pre `time > previous_update_time`
     * 
     * @param time The time of the update.
     */
    virtual void update(double time) = 0;

    /**
     * @brief Forecast the state at a time in the future.
     * 
     * @param time The time of the prediction.
     * @returns The predicted state.
     */
    virtual Eigen::VectorXd forecast(double time) = 0;
};

/**
 * @brief A trivial last observation carried forward (LOCF) forecast that
 * returns the most recent observation.
 */
class LOCFForecast : public Forecast
{
public:

    struct Configuration {

        /// The most recent observation.
        Eigen::VectorXd observation;

        /// JSON conversion for average forecast configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, observation);
    };

    /**
     * @brief A prediction only handle to a last observation carried forward
     * forecast.
     */
    class Handle : public Forecast::Handle
    {
    public:

        /**
         * @brief Get the last observation carried forward.
         * 
         * @param time The time of the observation, unused.
         * @returns The last observation.
         */
        inline Eigen::VectorXd forecast(double time) override {
            std::shared_lock lock(m_parent->m_mutex);
            return m_parent->m_observation;
        };

        /**
         * @brief Make a copy of the last observation carried forward forecast
         * handle.
         * 
         * Used to copy amongst dynamics objects for multi-threading.
         */
        inline std::unique_ptr<Forecast::Handle> copy() override {
            return std::unique_ptr<Forecast::Handle>(new Handle(*this));
        }

    private:

        friend class LOCFForecast;

        inline Handle(LOCFForecast *parent)
            : m_parent(parent)
        {}

        /// Pointer to the parent forecaster.
        LOCFForecast *m_parent;
    };

    /**
     * @brief Create a last observation carried forward forecast.
     * 
     * @param configuration The forecast configuration.
     * @return Pointer to the forecast on success or nullptr on failure.
     */
    static inline std::unique_ptr<LOCFForecast> create(
        const Configuration &configuration
    ) {
        return std::unique_ptr<LOCFForecast>(
            new LOCFForecast(configuration.observation)
        );
    }

    /**
     * @brief Create a handle to the last observation carried forward forecast.
     */
    inline std::unique_ptr<Forecast::Handle> create_handle() override {
        return std::unique_ptr<Handle>(new Handle(this));
    }

    /**
     * @brief Update the last observation.
     * 
     * @param measurement The measurement or observation.
     * @param time The time of the measurement, unused.
     */
    inline void update(Eigen::VectorXd measurement, double time) override {
        std::unique_lock lock(m_mutex);
        m_observation = measurement;
    }

    /**
     * @brief Updating without a reference does nothing.
     * @param time The time of the observation, unused.
     */
    inline void update(double time) override {}

    /**
     * @brief Get the last observation carried forward.
     * 
     * @param time The time of the observation, unused.
     * @returns The last observation.
     */
    inline Eigen::VectorXd forecast(double /* time */) override {
        std::shared_lock lock(m_mutex);
        return m_observation;
    }

private:

    LOCFForecast(Eigen::VectorXd observation)
        : m_observation(observation)
    {}

    /// Mutex protecting concurrent access to the last observation.
    std::shared_mutex m_mutex;

    /// The last observation.
    Eigen::VectorXd m_observation;
};

/**
 * @brief An average forecast that returns the average observation over an
 * averaging window, always including the most recent measurement even when
 * outside of the window (constant forecast).
 */
class AverageForecast : public Forecast
{
public:

    struct Configuration {

        /// The number of dimensions in each state.
        unsigned int dimensions;

        /// The period of time over which to calculate the average..
        double window;

        /// JSON conversion for average forecast configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, dimensions, window);
    };

    /**
     * @brief A prediction only handle to a average forecast.
     */
    class Handle : public Forecast::Handle
    {
    public:

        /**
         * @brief Yields the average observation.
         * 
         * @param time The time of the observation, unused.
         * @returns The average observation.
         */
        inline Eigen::VectorXd forecast(double time) override {
            std::shared_lock lock(m_parent->m_mutex);
            return m_parent->m_average;
        };

        /**
         * @brief Make a copy of the average forecast handle.
         * 
         * Used to copy amongst dynamics objects for multi-threading.
         */
        inline std::unique_ptr<Forecast::Handle> copy() override {
            return m_parent->create_handle();
        }

    private:

        friend class AverageForecast;

        /**
         * @brief Initialise a average forecast handle.
         * @param parent The owning average forecast.
         */
        Handle(AverageForecast *parent)
            : m_parent(parent)
        {}

        /// Pointer to the average forecast.
        std::unique_ptr<AverageForecast> m_parent;
    };

    /**
     * @brief Create an average forecast.
     * 
     * @param configuration 
     * @return std::unique_ptr<AverageForecast> 
     */
    static std::unique_ptr<AverageForecast> create(
        const Configuration &configuration
    );

    /**
     * @brief Create a handle to the average forecast.
     */
    inline std::unique_ptr<Forecast::Handle> create_handle() override {
        return std::unique_ptr<Handle>(new Handle(this));
    }

    /**
     * @brief Update the predictor with an observed measurement.
     * 
     * @param measurement The observed measurement.
     * @param time The time of the observation.
     */
    inline void update(Eigen::VectorXd measurement, double time) override;

    /**
     * @brief Does nothing.
     */
    inline void update(double time) override {}

    /**
     * @brief Predict the state as the last observed measurement, regardless of
     * time.
     * 
     * @param time The time of the forecasted state, unused.
     * @returns The average observation.
     */
    inline Eigen::VectorXd forecast(double /* time */) override;

private:

    friend class Handle;

    AverageForecast(unsigned int dimensions);

    /// Mutex protecting concurrent read / write.
    std::shared_mutex m_mutex;

    /// The window of forces to average over. The most recent observation is
    /// always  kept regardless of its age.
    std::vector<std::pair<double, Eigen::VectorXd>> m_buffer;

    /// The average observation of the window.
    Eigen::VectorXd m_average;
};

/**
 * @brief A forecast based on a kalman filter.
 * 
 * Note that update() must be called every configured time_step, with or without
 * an observation.
 */
class KalmanForecast : public Forecast
{
public:

    struct Configuration {

        /// The number of dimensions to forecast.
        unsigned int dimension;

        /// The time step of the state transition matrix.
        double time_step;

        /// The horison over which the forecast should be predicted.
        double horison;

        /// The order of the euclidean model.
        unsigned int order;

        /// E.g. {var(x), var(y), var(z), var(dx), var(dy), var(dz), ...}
        Eigen::VectorXd transition_variance;

        /// E.g. {var(x), var(y), var(z), var(dx), var(dy), var(dz), ...}
        Eigen::VectorXd observation_variance;

        /// JSON conversion for kalman forecast configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            dimension, time_step, horison, order, transition_variance,
            observation_variance
        );
    };

    /**
     * @brief A read only handle to a KalmanForecast.
     */
    class Handle : public Forecast::Handle
    {
    public:

        /**
         * @brief Predict the state at a time since the last kalman forecast
         * update.
         * 
         * @param time The time of the forecast.
         * @returns The predicted state.
         */
        inline Eigen::VectorXd forecast(double time) {
            return m_parent->forecast(time);
        }

        /**
         * @brief Create a copy of the kalman forecast handle.
         */
        inline std::unique_ptr<Forecast::Handle> copy() override {
            return m_parent->create_handle();
        }

    private:

        friend class KalmanForecast;

        /**
         * @brief Initialise a kalman forecast handle.
         * @param parent The kalman forecast.
         */
        Handle(KalmanForecast *parent)
            : m_parent(parent)
        {}

        /// Pointer to the kalman forecast.
        std::shared_ptr<KalmanForecast> m_parent;
    };

    /**
     * @brief Create a new kalman forecast.
     * 
     * @param configuration The configuration of the predictor.
     * @returns A pointer to the predictor on success or nullptr on failure.
     */
    static std::unique_ptr<KalmanForecast> create(
        const Configuration &configuration
    );

    /**
     * @brief Create a handle to the forecast.
     */
    inline std::unique_ptr<Forecast::Handle> create_handle() override {
        return std::unique_ptr<Handle>(new Handle(this));
    }

    /**
     * @brief Create a state transition matrix of a given order.
     * 
     * The order is the highest derivative in each observation.
     * 
     * @param time_step The time step of the transition matrix.
     * @param states The number of dimensions of the observed state.
     * @param order The order of the state transition matrix.
     * 
     * @returns The state transition matrix.
     */
    static Eigen::MatrixXd create_euler_state_transition_matrix(
        double time_step,
        unsigned int states,
        unsigned int order
    );

    /**
     * @brief Update the forecast with an observation.
     * 
     * @param measurement The observed measurement.
     * @param time The time of the observation.
     */
    inline void update(Eigen::VectorXd measurement, double time) override;

    /**
     * @brief Update without an observation.
     */
    inline void update(double time) override;

    /**
     * @brief Predict the force as the last observed force, regardless of
     * time.
     * 
     * @param time The time of the force prdiction
     * @returns The last observed force.
     */
    Eigen::VectorXd forecast(double /* time */) override;

private:

    friend class Handle;

    KalmanForecast() = default;

    /// The period of time over which the prediction is made.
    double m_horison;

    /// The expected kalman time step.
    double m_time_step;

    /// The number of steps to make in each forecast.
    unsigned int m_steps;

    /// The time of the last forecast.
    double m_last_update;

    std::shared_mutex m_mutex;

    /// The kalman filter used to estimate the current force.
    std::unique_ptr<KalmanFilter> m_kalman;

    /// The kalman filter used to estimate future forces.
    std::unique_ptr<KalmanFilter> m_filter;

    /// The calculated state, storing force derivatives.
    Eigen::VectorXd m_state;

    /// The latest updated horison.
    Eigen::MatrixXd m_prediction;
};

/**
 * @brief Configuration of a forecast.
 * 
 * Postpone configuration until types are fully declared.
 */
struct Forecast::Configuration
{
    std::variant<
        LOCFForecast::Configuration,
        AverageForecast::Configuration,
        KalmanForecast::Configuration
    > config;

    inline void to_json(json &j, const Forecast::Configuration &configuration)
    {
        if (std::holds_alternative<LOCFForecast::Configuration>(configuration.config)) {
            j["type"] = "constant";
            j["configuration"] = std::get<LOCFForecast::Configuration>(configuration.config);
        }
        else if (std::holds_alternative<AverageForecast::Configuration>(configuration.config)) {
            j["type"] = "average";
            j["configuration"] = std::get<AverageForecast::Configuration>(configuration.config);
        }
        else if (std::holds_alternative<KalmanForecast::Configuration>(configuration.config)) {
            j["type"] = "kalman";
            j["configuration"] = std::get<KalmanForecast::Configuration>(configuration.config);
        }
    }

    inline void from_json(const json &j, Forecast::Configuration &configuration)
    {
        auto type = j.at("type").get<std::string>();
        if (type == "constant") {
            configuration.config = (LOCFForecast::Configuration)j.at("configuration");
        }
        else if (type == "average") {
            configuration.config = (AverageForecast::Configuration)j.at("configuration");
        }
        else if (type == "kalman") {
            configuration.config = (KalmanForecast::Configuration)j.at("configuration");
        }
    }
};
