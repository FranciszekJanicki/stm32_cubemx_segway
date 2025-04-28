#ifndef SEGWAY_EVENT_BIT_HPP
#define SEGWAY_EVENT_BIT_HPP

#include <cstdint>
#include <utility>

namespace segway {

    struct IMUEventBit {
        enum : std::uint8_t {
            DATA_READY = 1 << 0,
            I2C_ERROR = 1 << 1,
            SAMPLING_TIMER = 1 << 2,
            TX_COMPLETE = 1 << 3,
            RX_COMPLETE = 1 << 4,
            ALL = (DATA_READY | I2C_ERROR | SAMPLING_TIMER | TX_COMPLETE | RX_COMPLETE),
        };
    };

    struct WheelEventBit {
        enum : std::uint8_t {
            LEFT_PWM_PULSE = 1 << 0,
            RIGHT_PWM_PULSE = 1 << 1,
            LEFT_STEP_TIMER = 1 << 2,
            RIGHT_STEP_TIMER = 1 << 3,
            ALL = (LEFT_PWM_PULSE | RIGHT_PWM_PULSE | LEFT_STEP_TIMER | RIGHT_STEP_TIMER),
        };
    };

}; // namespace segway

#endif // SEGWAY_EVENT_BIT_HPP