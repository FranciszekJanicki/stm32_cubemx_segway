#ifndef WHEELS_HPP
#define WHEELS_HPP

#include "wheel.hpp"
#include <array>

namespace Segway {

    using Wheels = std::array<Wheel, 2UL>;

    void update_wheel_step_count(Wheels& wheels, WheelType const wheel_type) noexcept;

    void set_wheel_speed(Wheels& wheels,
                         WheelType const wheel_type,
                         std::float64_t const wheel_speed,
                         std::float64_t const dt) noexcept;

    void set_wheels_speed(Wheels& wheels,
                          std::float64_t const left_wheel_speed,
                          std::float64_t const right_wheel_speed,
                          std::float64_t const dt) noexcept;

    std::float64_t get_wheel_speed(Wheels& wheels, WheelType const wheel_type, std::float64_t const dt) noexcept;

    void set_wheel_position(Wheels& wheels,
                            WheelType const wheel_type,
                            std::float64_t const wheel_position,
                            std::float64_t const dt) noexcept;

    void set_wheels_position(Wheels& wheels,
                             std::float64_t const left_wheel_position,
                             std::float64_t const right_wheel_position,
                             std::float64_t const dt) noexcept;

    std::float64_t get_wheel_position(Wheels& wheels, WheelType const wheel_type, std::float64_t const dt) noexcept;

    std::float64_t
    get_wheel_diff_rotation(Wheels& wheels, std::float64_t const dt, std::float64_t const wheel_distance) noexcept;

    std::float64_t get_wheel_diff_position(Wheels& wheels, std::float64_t const dt) noexcept;

    WheelDriver& get_wheel_driver(Wheels& wheels, WheelType const wheel_type) noexcept;

}; // namespace Segway

#endif // WHEELS_HPP