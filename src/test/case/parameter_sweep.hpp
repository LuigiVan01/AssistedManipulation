#pragma once

#include "test/case/external_wrench.hpp"

class ParameterSweep : public RegisteredTest<ParameterSweep>
{
public:

    /**
     * @brief A parameter in the ExternalWrenchTest to sweep.
     */
    struct Parameter {

        /**
         * @brief A json pointer into the external trajectory configuration
         * structure to perform a sweep on.
         * 
         * See the default configuration for examples, and the documentation
         * here: https://json.nlohmann.me/features/json_pointer/
         */
        std::string pointer;

        /// The minimum values of the parameter.
        double minimum;

        /// The maximum value of the parameter.
        double maximum;

        /// The increment of the parameter.
        double step;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Parameter,
            pointer, minimum, maximum, step
        );
    };

    struct Configuration {

        /// The default configuration to 
        ExternalWrenchTest::Configuration base;

        /// The parameters to sweep over.
        std::vector<Parameter> parameters;

        // JSON conversion for parameter sweep configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration, base, parameters
        )
    };

    static const Configuration DEFAULT_CONFIGURATION;

    static std::unique_ptr<ParameterSweep> create(
        Options &options
    );

    static std::unique_ptr<ParameterSweep> create(
        const Configuration &configuration
    );

    bool run() override;

private:

    ParameterSweep(const Configuration &configuration);

    /// The configuration to apply the patch to.
    ExternalWrenchTest::Configuration default_configuration;

    /// The parameters being iterated over with a cartesian product.
    std::vector<Parameter> m_parameters;

    /// The current item from the cartesian product of the parameters being
    /// tested.
    std::vector<double> m_current;
};
