#ifndef SEGWAY_EVENT_BIT_HPP
#define SEGWAY_EVENT_BIT_HPP

#include <cstdint>
#include <utility>

namespace segway {

    struct IMUEventBit {
        enum : std::uint32_t {
            START = 1 << 0,
            STOP = 1 << 1,
            DATA_READY = 1 << 2,
            I2C_ERROR = 1 << 3,
            TX_COMPLETE = 1 << 4,
            RX_COMPLETE = 1 << 5,
            ALL = (START | STOP | DATA_READY | I2C_ERROR | TX_COMPLETE | RX_COMPLETE),
        };
    };

    struct ControlEventBit {
        enum : std::uint32_t {
            START = 1 << 0,
            STOP = 1 << 1,
            ALL = (START | STOP),
        };
    };

    struct WheelEventBit {
        enum : std::uint32_t {
            START = 1 << 0,
            STOP = 1 << 1,
            LEFT_STEP_TIMER = 1 << 2,
            RIGHT_STEP_TIMER = 1 << 3,
            ALL = (START | STOP | LEFT_STEP_TIMER | RIGHT_STEP_TIMER),
        };
    };

}; // namespace segway

#endif // SEGWAY_EVENT_BIT_HPP