#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "step_driver.hpp"
#include <cstdint>

namespace Segway {

    enum struct WheelType : std::uint8_t {
        LEFT,
        RIGHT,
        NONE,
    };

    char const* wheel_type_to_string(WheelType const wheel_type) noexcept;

    using Driver = StepDriver::StepDriver;

    struct WheelDriver {
        std::float32_t get_wheel_position(this WheelDriver& self, std::float32_t const dt) noexcept;

        std::float32_t get_wheel_speed(this WheelDriver& self, std::float32_t const dt) noexcept;

        void
        set_wheel_speed(this WheelDriver& self, std::float32_t const wheel_speed, std::float32_t const dt) noexcept;

        Driver driver = {};
        std::float32_t wheel_radius = 0.0F;
    };

    struct Wheel {
        WheelType type = {};
        WheelDriver driver = {};
    };

}; // namespace Segway

#endif // WHEEL_HPP