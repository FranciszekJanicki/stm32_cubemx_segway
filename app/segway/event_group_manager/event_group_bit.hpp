#ifndef SEGWAY_EVENT_BIT_HPP
#define SEGWAY_EVENT_BIT_HPP

#include <cstdint>
#include <utility>

namespace segway {

    struct IMUEventBit {
        enum : std::uint32_t {
            DATA_READY = 1 << 0,
            I2C_ERROR = 1 << 1,
            TX_COMPLETE = 1 << 2,
            RX_COMPLETE = 1 << 3,
            ALL = (DATA_READY | I2C_ERROR | TX_COMPLETE | RX_COMPLETE),
        };
    };

    struct ControlEventBit {
        enum : std::uint32_t {};
    };

    struct WheelEventBit {
        enum : std::uint32_t {
            LEFT_STEP_TIMER = 1 << 0,
            RIGHT_STEP_TIMER = 1 << 1,
            ALL = (LEFT_STEP_TIMER | RIGHT_STEP_TIMER),
        };
    };

}; // namespace segway

#endif // SEGWAY_EVENT_BIT_HPP