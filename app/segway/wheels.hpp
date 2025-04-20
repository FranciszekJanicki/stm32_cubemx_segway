#ifndef WHEELS_HPP
#define WHEELS_HPP

#include "wheel.hpp"
#include <array>

namespace Segway {

    struct Wheels {
    public:
        void update_wheel_step_count(this Wheels& self, WheelType const wheel_type) noexcept;

        void set_wheel_speed(this Wheels& self,
                             WheelType const wheel_type,
                             std::float32_t const wheel_speed,
                             std::float32_t const dt) noexcept;

        std::float32_t get_wheel_speed(this Wheels& self, WheelType const wheel_type, std::float32_t const dt) noexcept;

        void set_wheel_position(this Wheels& self,
                                WheelType const wheel_type,
                                std::float32_t const wheel_position,
                                std::float32_t const dt) noexcept;

        std::float32_t
        get_wheel_position(this Wheels& self, WheelType const wheel_type, std::float32_t const dt) noexcept;

        std::float32_t get_wheel_diff_rotation(this Wheels& self, std::float32_t const dt) noexcept;
        std::float32_t get_wheel_diff_position(this Wheels& self, std::float32_t const dt) noexcept;

        std::float32_t wheel_base_len = {};
        std::array<Wheel, 2UL> wheels = {};

    private:
        WheelDriver& get_wheel_driver(this Wheels& self, WheelType const wheel_type) noexcept;
    };

}; // namespace Segway

#endif // WHEELS_HPP