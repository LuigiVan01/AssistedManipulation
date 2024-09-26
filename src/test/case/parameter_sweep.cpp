#include "test/case/parameter_sweep.hpp"

const ParameterSweep::Configuration ParameterSweep::DEFAULT_CONFIGURATION {
    .base = ExternalWrenchTest::DEFAULT_CONFIGURATION,
    .parameters = {
        ParameterSweep::Parameter {
            .pointer = "/base/objective/assisted_manipulation/maximum_power/constant_cost",
            .minimum = 500,
            .maximum = 2500,
            .step = 500
        },
        ParameterSweep::Parameter {
            .pointer = "/base/objective/assisted_manipulation/maximum_power/quadratic_cost",
            .minimum = 500,
            .maximum = 2500,
            .step = 500
        }
    }
};

std::unique_ptr<ParameterSweep> ParameterSweep::create(
    Options &options
) {

}

std::unique_ptr<ParameterSweep> ParameterSweep::create(
    const Configuration &configuration
) {

}

bool ParameterSweep::run()
{
    for (;;) {

        // Check the ending condition.
        for (int i = 0; i < m_parameters.size(); i++) {
            auto &parameter = m_parameters[i];

            if (m_current[i] == parameter.maximum)
        }
        for (auto &parameter : m_parameters) {
            if (m_current.)
        }
    }

    return true;
}
