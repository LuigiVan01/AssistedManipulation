#include "test/case/circle.hpp"
#include "test/case/power_minimisation.hpp"

std::unique_ptr<Test> PowerMinimisation::create(Options &options)
{
    auto test = std::unique_ptr<PowerMinimisation>(new PowerMinimisation());
    test->m_folder = options.folder;
    test->m_duration = options.duration;
    return test;
}

bool PowerMinimisation::run()
{
    Circle::Configuration configuration = Circle::DEFAULT_CONFIGURATION;
    configuration.folder = "power_minimisation" / configuration.folder;
    configuration.duration = 10.0;

    // Cost based on quadratic term only.
    configuration.objective.enable_minimise_power = true;
    configuration.objective.maximum_power.limit = 0.0;
    configuration.objective.maximum_power.constant_cost = 0.0;

    {
        configuration.objective.maximum_power.quadratic_cost = 0;
        configuration.folder = m_folder / "no_power";
        auto test = Circle::create(configuration);
        if (!test || !test->run())
            return false;
    }

    {
        configuration.objective.maximum_power.quadratic_cost = 1000;
        configuration.folder = m_folder / "power_1000";
        auto test = Circle::create(configuration);
        if (!test || !test->run())
            return false;
    }

    {
        configuration.objective.maximum_power.quadratic_cost = 100000;
        configuration.folder = m_folder / "power_100000";
        auto test = Circle::create(configuration);
        if (!test || !test->run())
            return false;
    }

    return true;
}
