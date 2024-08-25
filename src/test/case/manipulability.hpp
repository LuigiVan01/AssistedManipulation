#pragma once

#include "test/test.hpp"

class ManipulabilityTest : public RegisteredTest<ManipulabilityTest>
{
public:

    struct Configuration {

        std::string folder;

        double duration;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, folder)
    };

    static inline constexpr const char *TEST_NAME = "manipulability";

    static std::unique_ptr<Test> create(Options &options);

    bool run() override;

private:

    ManipulabilityTest() = default;

    double m_duration;

    std::filesystem::path m_folder;
};
