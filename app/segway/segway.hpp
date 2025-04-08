#ifndef SEGWAY_HPP
#define SEGWAY_HPP

#include "segway_config.hpp"
#include "wheel.hpp"
#include <array>

namespace Segway {

    struct Segway {
        void update_step_count(this Segway& self, WheelType const wheel_type) noexcept;

        void
        operator()(this Segway& self, std::array<std::float32_t, 6UL> const& x_ref, std::float32_t const dt) noexcept;

        void
        run_segway(this Segway& self, std::array<std::float32_t, 6UL> const& x_ref, std::float32_t const dt) noexcept;

        IMU sensor = {};
        SFR regulator = {};
        SFO observer = {};
        Config config = {};
        std::array<Wheel, 2UL> wheels = {};

    private:
        std::float32_t get_tilt(this Segway& self) noexcept;
        std::float32_t get_rotation(this Segway& self) noexcept;
        std::float32_t get_position(this Segway& self, std::uint32_t const dt) noexcept;

        std::float32_t
        get_wheel_position(this Segway& self, WheelType const wheel_type, std::float32_t const dt) noexcept;
        void set_wheel_speed(this Segway& self,
                             WheelType const wheel_type,
                             std::float32_t const wheel_speed,
                             std::float32_t dt) noexcept;

        WheelDriver& get_wheel_driver(this Segway& self, WheelType const wheel_type) noexcept;
    };

}; // namespace Segway

#endif // SEGWAY_HPP