#ifndef SEGWAY_EVENT_GROUP_BIT_HPP
#define SEGWAY_EVENT_GROUP_BIT_HPP

#include <cstdint>

namespace segway {

    enum IMUEventGroupBit : std::uint8_t {
        IMU_DATA_READY,
        IMU_I2C_ERROR,
        IMU_SAMPLING_TIMER,
        IMU_TX_COMPLETE,
        IMU_RX_COMPLETE,
        IMU_ALL = (IMU_DATA_READY | IMU_I2C_ERROR | IMU_SAMPLING_TIMER | IMU_TX_COMPLETE | IMU_RX_COMPLETE),
    };

    enum WheelEventGroupBit : std::uint8_t {
        WHEEL_LEFT_PWM_PULSE,
        WHEEL_RIGHT_PWM_PULSE,
        WHEEL_LEFT_STEP_TIMER,
        WHEEL_RIGHT_STEP_TMER,
        ALL = (WHEEL_LEFT_PWM_PULSE | WHEEL_RIGHT_PWM_PULSE | WHEEL_LEFT_STEP_TIMER | WHEEL_RIGHT_STEP_TMER),
    };

}; // namespace segway

#endif // SEGWAY_EVENT_GROUP_BIT_HPP