#pragma once

#include <cstdlib>
#include <filesystem>

#include "test/case/circle.hpp"
#include "frankaridgeback/objective/assisted_manipulation.hpp"

class PowerMinimisation : public RegisteredTest<PowerMinimisation>
{
public:

    struct Configuration {

        std::string folder;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, folder)
    };

    static inline constexpr const char *TEST_NAME = "power_minimisation";

    /**
     * @brief Create a circle test instance.
     * 
     * @param options The test options. The configuration overrides from the
     * default configuration.
     * 
     * @returns A pointer to the test on success or nullptr on failure.
     */
    static std::unique_ptr<Test> create(Options &options);

    /**
     * @brief Run the test.
     * @returns If the test was successful.
     */
    bool run() override;

private:

    PowerMinimisation() = default;

    double m_duration;

    std::filesystem::path m_folder;
};
