#include <chrono>

namespace mppi {

/**
 * @brief The clock used to keep track of time.
 */
using Clock = std::chrono::high_resolution_clock;

/**
 * @brief The time in seconds.
 */
using Time = std::chrono::seconds;

/**
 * @brief A duration in seconds.
 */
using Duration = std::chrono::seconds;

/**
 * @brief Get the current time.
 * 
 * @returns The timestamp at the current time.
 */
inline Time now() {
    return Clock::now().time_since_epoch();
}

} // namespace mppi
