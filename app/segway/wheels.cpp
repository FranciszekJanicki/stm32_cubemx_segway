#include "wheels.hpp"

namespace Segway {

    void update_wheel_step_count(Wheels& wheels, WheelType const wheel_type) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        wheel_driver.driver.update_step_count();
    }

    void set_wheel_speed(Wheels& wheels,
                         WheelType const wheel_type,
                         std::float32_t const wheel_speed,
                         std::float32_t const dt) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        wheel_driver.set_wheel_speed(wheel_speed, dt);
    }

    void set_wheels_speed(Wheels& wheels,
                          std::float32_t const left_wheel_speed,
                          std::float32_t const right_wheel_speed,
                          std::float32_t const dt) noexcept
    {
        auto& left_wheel_driver = get_wheel_driver(wheels, WheelType::LEFT);
        left_wheel_driver.set_wheel_speed(left_wheel_speed, dt);

        auto& right_wheel_driver = get_wheel_driver(wheels, WheelType::RIGHT);
        right_wheel_driver.set_wheel_speed(right_wheel_speed, dt);
    }

    std::float32_t get_wheel_speed(Wheels& wheels, WheelType const wheel_type, std::float32_t const dt) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        return wheel_driver.get_wheel_speed(dt);
    }

    void set_wheel_position(Wheels& wheels,
                            WheelType const wheel_type,
                            std::float32_t const wheel_position,
                            std::float32_t const dt) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        wheel_driver.set_wheel_position(wheel_position, dt);
    }

    void set_wheels_position(Wheels& wheels,
                             std::float32_t const left_wheel_position,
                             std::float32_t const right_wheel_position,
                             std::float32_t const dt) noexcept
    {
        auto& left_wheel_driver = get_wheel_driver(wheels, WheelType::LEFT);
        left_wheel_driver.set_wheel_position(left_wheel_position, dt);

        auto& right_wheel_driver = get_wheel_driver(wheels, WheelType::RIGHT);
        right_wheel_driver.set_wheel_position(right_wheel_position, dt);
    }

    std::float32_t get_wheel_position(Wheels& wheels, WheelType const wheel_type, std::float32_t const dt) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        return wheel_driver.get_wheel_position(dt);
    }

    std::float32_t
    get_wheel_diff_rotation(Wheels& wheels, std::float32_t const dt, std::float32_t const wheel_distance) noexcept
    {
        auto& left_wheel_driver = get_wheel_driver(wheels, WheelType::LEFT);
        auto& right_wheel_driver = get_wheel_driver(wheels, WheelType::RIGHT);

        return (left_wheel_driver.get_wheel_position(dt) / left_wheel_driver.wheel_radius -
                right_wheel_driver.get_wheel_position(dt) / right_wheel_driver.wheel_radius) /
               wheel_distance;
    }

    std::float32_t get_wheel_diff_position(Wheels& wheels, std::float32_t const dt) noexcept
    {
        auto& left_wheel_driver = get_wheel_driver(wheels, WheelType::LEFT);
        auto& right_wheel_driver = get_wheel_driver(wheels, WheelType::RIGHT);

        return left_wheel_driver.get_wheel_position(dt) - right_wheel_driver.get_wheel_position(dt);
    }

    WheelDriver& get_wheel_driver(Wheels& wheels, WheelType const wheel_type) noexcept
    {
        auto* wheel =
            std::ranges::find_if(wheels, [wheel_type](Wheel const& wheel) { return wheel.type == wheel_type; });

        return wheel->driver;
    }

}; // namespace Segway