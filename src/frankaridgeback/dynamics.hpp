#pragma once

#include "controller/mppi.hpp"
#include "frankaridgeback/dof.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

class Dynamics : public mppi::Dynamics
{
public:

    static inline std::filesystem::path find_path() {
        return (std::filesystem::current_path() / "model/robot.urdf").string();
    }

    virtual void set_end_effector_force(Eigen::Ref<Eigen::Vector3d> force) = 0;

    virtual double get_power() = 0;

    virtual Eigen::Quaterniond get_end_effector_orientation() = 0;

    virtual Eigen::Vector3d get_end_effector_position() = 0;

    virtual Eigen::Vector3d get_end_effector_velocity() = 0;

    virtual Eigen::Ref<Eigen::Matrix<double, 6, DoF::JOINTS>> get_end_effector_jacobian() = 0;
};

} // namespace FrankaRidgeback
