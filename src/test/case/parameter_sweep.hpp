#pragma once

#include "test/test.hpp"

class ParameterSweep : public RegisteredTest<ParameterSweep>
{
public:

    struct Parameter {

        std::string parameter;

        double minimum;

        double maximum;

        double step;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Parameter, minimum, maximum, step);
    };

    struct Configuration {

        std::vector<Parameter> parameters;
    };

    bool run() override;

private:

};
