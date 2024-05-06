#include <mutex>

#include "controller.hpp"

class PointCost : public controller::Cost<3, ControlDoF>
{
public:

    PointCost()

    /**
     * @brief Update the cost given a subsequent state from a contorl input.
     * 
     * @param state The next state of the system.
     * @param control The controls used to achieve the state.
     * @param t The change in time.
     */
    double step(
        const Eigen::Matrix<double, StateDoF, 1> &state,
        const Eigen::Matrix<double, ControlDoF, 1> &control,
        double dt
    );

    /**
     * @brief Unused.
     */
    void reset() override {};

private:

    /// The point in space for the end effector to reach.
    Eigen::Vector3d m_point;
};
