#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "step_driver.hpp"
#include <cstdint>

namespace segway {

    using Driver = step_driver::StepDriver;

    enum struct WheelType : std::uint8_t {
        LEFT,
        RIGHT,
        NONE,
    };

    char const* wheel_type_to_string(WheelType const wheel_type) noexcept;

    struct WheelDriver {
        void initialize(this WheelDriver& self) noexcept;
        void deinitialize(this WheelDriver& self) noexcept;

        void start(this WheelDriver& self) noexcept;
        void stop(this WheelDriver& self) noexcept;

        void update_step_count(this WheelDriver& self) noexcept;

        std::float64_t get_wheel_position(this WheelDriver& self, std::float64_t const dt) noexcept;

        void set_wheel_position(this WheelDriver& self,
                                std::float64_t const wheel_position,
                                std::float64_t const dt) noexcept;

        std::float64_t get_wheel_speed(this WheelDriver& self, std::float64_t const dt) noexcept;

        void set_wheel_speed(this WheelDriver& self,
                             std::float64_t const wheel_speed,
                             std::float64_t const dt) noexcept;

        Driver driver = {};
        std::float64_t wheel_radius = 0.0F;
    };

    struct Wheel {
        WheelType type = {};
        WheelDriver driver = {};
    };

}; // namespace segway

#endif // WHEEL_HPP