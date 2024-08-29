#pragma once

#include <cstddef>

namespace FrankaRidgeback {

// Various degrees of freedom of the franka research 3 ridgeback.
namespace DoF {

    // Base degrees of freedom.
    constexpr const std::size_t
        BASE_X = 1,
        BASE_Y = 1,
        BASE_YAW = 1,
        BASE_POSITION = BASE_X + BASE_Y,
        BASE = BASE_X + BASE_Y + BASE_YAW;

    // Arm degrees of freedom.
    constexpr const std::size_t
        ARM_JOINT1 = 1,
        ARM_JOINT2 = 1,
        ARM_JOINT3 = 1,
        ARM_JOINT4 = 1,
        ARM_JOINT5 = 1,
        ARM_JOINT6 = 1,
        ARM_JOINT7 = 1,
        ARM = ARM_JOINT1 + ARM_JOINT2 + ARM_JOINT3 + ARM_JOINT4 + ARM_JOINT5 + ARM_JOINT6 + ARM_JOINT7;

    // Gripper degrees of freedom.
    constexpr const std::size_t
        GRIPPER_LEFT = 1,
        GRIPPER_RIGHT = 1,
        GRIPPER = GRIPPER_LEFT + GRIPPER_RIGHT;

    // Joint dimensions.
    constexpr const std::size_t
        JOINTS = BASE + ARM /* + GRIPPER */;

    // End effector force.
    constexpr const std::size_t
        EXTERNAL_FORCE_X = 1,
        EXTERNAL_FORCE_Y = 1,
        EXTERNAL_FORCE_Z = 1,
        EXTERNAL_FORCE = EXTERNAL_FORCE_X + EXTERNAL_FORCE_Y + EXTERNAL_FORCE_Z;

    // End effector torque.
    constexpr const std::size_t
        EXTERNAL_TORQUE_X = 1,
        EXTERNAL_TORQUE_Y = 1,
        EXTERNAL_TORQUE_Z = 1,
        EXTERNAL_TORQUE = EXTERNAL_TORQUE_X + EXTERNAL_TORQUE_Y + EXTERNAL_TORQUE_Z;

    // End effector wrench.
    constexpr const std::size_t
        EXTERNAL_WRENCH = EXTERNAL_FORCE + EXTERNAL_TORQUE;

    // Available energy in the energy tank.
    constexpr const std::size_t AVAILABLE_ENERGY = 1;

    /**
     * @brief Joint position, joint velocity, external torque and tank energy.
     */
    constexpr const std::size_t STATE  = 2 * JOINTS + EXTERNAL_WRENCH + AVAILABLE_ENERGY;

    /**
     * @brief Base velocity, arm torque and gripper position.
     * 
     * See FrankaRidgeback::Control
     */
    constexpr const std::size_t CONTROL = JOINTS;

} // namespace DoF

} // namespace FrankaRidgeback
