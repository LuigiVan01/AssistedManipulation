#include "logging/pid.hpp"

namespace logger {

std::unique_ptr<PID> PID::create(PID::Configuration &&configuration)
{
    using namespace std::string_literals;
    std::vector<std::string> state;
    for (int i = 1; i < configuration.reference_dof + 1; i++)
        state.push_back("state"s + std::to_string(i));

    std::vector<std::string> control;
    for (int i = 1; i < configuration.control_dof + 1; i++)
        control.push_back("control"s + std::to_string(i));

    auto pid = std::make_unique<PID>();

    if (configuration.reference) {
        pid->m_reference = CSV::create(CSV::Configuration{
            .path = configuration.folder / "reference.csv",
            .header = CSV::make_header("time", state)
        });
    }

    if (configuration.error) {
        pid->m_error = CSV::create(CSV::Configuration{
            .path = configuration.folder / "error.csv",
            .header = CSV::make_header("time", state)
        });
    }

    if (configuration.cumulative_error) {
        pid->m_cumulative_error = CSV::create(CSV::Configuration{
            .path = configuration.folder / "cumulative_error.csv",
            .header = CSV::make_header("time", state)
        });
    }

    if (configuration.saturation) {
        pid->m_saturation = CSV::create(CSV::Configuration{
            .path = configuration.folder / "saturation.csv",
            .header = CSV::make_header("time", control)
        });
    }

    bool error = (
        (configuration.reference && !pid->m_reference) ||
        (configuration.error && !pid->m_error) ||
        (configuration.cumulative_error && !pid->m_cumulative_error) ||
        (configuration.saturation && !pid->m_saturation)
    );

    if (error) {
        std::cerr << "failed to create csv logger" << std::endl;
        return nullptr;
    }

    return pid;
}

void PID::log(const controller::PID &pid)
{
    double time = pid.get_time();

    if (m_reference)
        m_reference->write(time, pid.get_reference());

    if (m_error)
        m_error->write(time, pid.get_error());

    if (m_cumulative_error)
        m_cumulative_error->write(time, pid.get_cumulative_error());

    if (m_saturation)
        m_saturation->write(time, pid.get_saturation());
}

} // namespace logger
