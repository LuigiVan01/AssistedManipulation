#include "frankaridgeback/safety.hpp"

namespace FrankaRidgeback {

std::unique_ptr<TrajectorySafetyFilter> TrajectorySafetyFilter::create(
    Configuration &&configuration
) {

}

Eigen::VectorXd TrajectorySafetyFilter::filter(
    Eigen::Ref<Eigen::VectorXd> state,
    Eigen::Ref<Eigen::VectorXd> control,
    double time
) {
    
}

void TrajectorySafetyFilter::reset(Eigen::Ref<Eigen::VectorXd> state, double time)
{

}

} // namespace FrankaRidgeback
