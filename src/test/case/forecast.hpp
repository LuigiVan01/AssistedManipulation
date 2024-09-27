#pragma once

#include "test/test.hpp"

/**
 * @brief Test for forecasting functionality.
 */
class ForecastTest : public RegisteredTest<ForecastTest>
{
public:

    static inline constexpr const char *TEST_NAME = "forecast";

    /**
     * @brief Create an instance of the forecast test.
     * 
     * @param options The options for the test. Unused.
     * @returns A pointer to the test on success or nullptr on failure.
     */
    inline static std::unique_ptr<ForecastTest> create(Options &options)
    {
        return std::unique_ptr<ForecastTest>(
            new ForecastTest(options.folder)
        );
    }

    /**
     * @brief Run the test.
     */
    bool run() override;

private:

    ForecastTest(std::filesystem::path folder);

    bool test_locf_forecast();

    bool test_average_forecast();

    bool test_kalman_linear_forecast();

    bool test_kalman_quadratic_forecast();

    /// Folder to write test results to.
    std::filesystem::path m_folder;
};
