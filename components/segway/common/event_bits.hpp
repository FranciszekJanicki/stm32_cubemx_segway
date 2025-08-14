#ifndef SEGWAY_EVENT_BITS_HPP
#define SEGWAY_EVENT_BITS_HPP

#include <cstdint>

namespace segway {

    enum ImuEventBit : std::uint32_t {
        IMU_EVENT_BIT_START = 1 << 0,
        IMU_EVENT_BIT_STOP = 1 << 1,
        IMU_EVENT_BIT_DATA_READY = 1 << 2,
        IMU_EVENT_BIT_I2C_ERROR = 1 << 3,
        IMU_EVENT_BIT_TX_COMPLETE = 1 << 4,
        IMU_EVENT_BIT_RX_COMPLETE = 1 << 5,
        IMU_EVENT_BIT_ALL =
            (IMU_EVENT_BIT_START | IMU_EVENT_BIT_STOP | IMU_EVENT_BIT_DATA_READY |
             IMU_EVENT_BIT_I2C_ERROR | IMU_EVENT_BIT_TX_COMPLETE | IMU_EVENT_BIT_RX_COMPLETE),
    };

    enum ControlEventBit : std::uint32_t {
        CONTROL_EVENT_BIT_START = 1 << 0,
        CONTROL_EVENT_BIT_STOP = 1 << 1,
        CONTROL_EVENT_BIT_ALL = (CONTROL_EVENT_BIT_START | CONTROL_EVENT_BIT_STOP),
    };

    enum WheelEventBit : std::uint32_t {
        WHEEL_EVENT_BIT_START = 1 << 0,
        WHEEL_EVENT_BIT_STOP = 1 << 1,
        WHEEL_EVENT_BIT_LEFT_STEP_TIMER = 1 << 2,
        WHEEL_EVENT_BIT_RIGHT_STEP_TIMER = 1 << 3,
        WHEEL_EVENT_BIT_ALL = (WHEEL_EVENT_BIT_START | WHEEL_EVENT_BIT_STOP |
                               WHEEL_EVENT_BIT_LEFT_STEP_TIMER | WHEEL_EVENT_BIT_RIGHT_STEP_TIMER),
    };

}; // namespace segway

#endif // SEGWAY_EVENT_BITS_HPP