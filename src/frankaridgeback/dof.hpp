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
        JOINTS = BASE + ARM + GRIPPER;

    // External torque.
    constexpr const std::size_t
        EXTERNAL_TORQUE = 1;

    /**
     * @brief Joint position, joint velocity and external torque.
     */
    constexpr const std::size_t STATE  = 2 * JOINTS + EXTERNAL_TORQUE;

    /**
     * @brief Base velocity, arm torque and gripper position.
     * 
     * See FrankaRidgeback::Control
     */
    constexpr const std::size_t CONTROL = JOINTS;

} // namespace DoF

} // namespace FrankaRidgeback
