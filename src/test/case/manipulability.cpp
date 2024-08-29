#include "test/case/base/no_manipulation.hpp"
#include "test/case/manipulability.hpp"

std::unique_ptr<Test> ManipulabilityTest::create(Options &options)
{
    auto test = std::unique_ptr<ManipulabilityTest>(new ManipulabilityTest());
    test->m_folder = options.folder;
    test->m_duration = options.duration;
    return test;
}

bool ManipulabilityTest::run()
{
    NoManipulation::Configuration configuration = NoManipulation::DEFAULT_CONFIGIURATION;
    configuration.folder = m_folder;
    configuration.duration = 60;

    configuration.actor.mppi.threads = 1;

    configuration.actor.controller_rate = 0.01;
    configuration.actor.controller_substeps = 1;
    configuration.actor.mppi.horison = 0.5;
    configuration.actor.mppi.time_step = 0.01;

    // Cost based on quadratic term only.
    configuration.objective.enable_maximise_manipulability = true;
    configuration.objective.minimum_manipulability.limit = 0.0;
    configuration.objective.minimum_manipulability.quadratic_cost = 100;

    // configuration.objective.enable_minimise_power = true;
    // configuration.objective.maximum_power.limit = 0.0;
    // configuration.objective.maximum_power.quadratic_cost = 1000.0;

    auto test = NoManipulation::create(configuration);
    if (!test || !test->run())
        return false;

    return true;
}
