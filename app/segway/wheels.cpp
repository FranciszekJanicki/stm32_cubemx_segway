#include "wheels.hpp"
#include "log.hpp"

namespace {

    constexpr auto TAG = "Wheels";

};

namespace Segway {

    void update_wheel_step_count(Wheels& wheels, WheelType const wheel_type) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        wheel_driver.driver.update_step_count();

        LOG(TAG, "Updating wheel %s step count!", wheel_type_to_string(wheel_type));
    }

    void set_wheel_speed(Wheels& wheels,
                         WheelType const wheel_type,
                         std::float64_t const wheel_speed,
                         std::float64_t const dt) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        wheel_driver.set_wheel_speed(wheel_speed, dt);

        LOG(TAG, "Updating wheel %s speed to: %f!", wheel_type_to_string(wheel_type), wheel_speed);
    }

    void set_wheels_speed(Wheels& wheels,
                          std::float64_t const left_wheel_speed,
                          std::float64_t const right_wheel_speed,
                          std::float64_t const dt) noexcept
    {
        auto& left_wheel_driver = get_wheel_driver(wheels, WheelType::LEFT);
        left_wheel_driver.set_wheel_speed(left_wheel_speed, dt);

        auto& right_wheel_driver = get_wheel_driver(wheels, WheelType::RIGHT);
        right_wheel_driver.set_wheel_speed(right_wheel_speed, dt);

        LOG(TAG, "Updating right wheel speed to: %f!", right_wheel_speed);
        LOG(TAG, "Updating left wheel speed to: %f!", left_wheel_speed);
    }

    std::float64_t get_wheel_speed(Wheels& wheels, WheelType const wheel_type, std::float64_t const dt) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        auto wheel_speed = wheel_driver.get_wheel_speed(dt);

        LOG(TAG, "Measured wheel %s speed of: %f!", wheel_type_to_string(wheel_type), wheel_speed);
        return wheel_speed;
    }

    void set_wheel_position(Wheels& wheels,
                            WheelType const wheel_type,
                            std::float64_t const wheel_position,
                            std::float64_t const dt) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        wheel_driver.set_wheel_position(wheel_position, dt);

        LOG(TAG, "Updating wheel %s position to: %f", wheel_type_to_string(wheel_type), wheel_position);
    }

    void set_wheels_position(Wheels& wheels,
                             std::float64_t const left_wheel_position,
                             std::float64_t const right_wheel_position,
                             std::float64_t const dt) noexcept
    {
        auto& left_wheel_driver = get_wheel_driver(wheels, WheelType::LEFT);
        left_wheel_driver.set_wheel_position(left_wheel_position, dt);

        auto& right_wheel_driver = get_wheel_driver(wheels, WheelType::RIGHT);
        right_wheel_driver.set_wheel_position(right_wheel_position, dt);

        LOG(TAG, "Updating right wheel speed to: %f!", right_wheel_position);
        LOG(TAG, "Updating left wheel speed to: %f!", left_wheel_position);
    }

    std::float64_t get_wheel_position(Wheels& wheels, WheelType const wheel_type, std::float64_t const dt) noexcept
    {
        auto& wheel_driver = get_wheel_driver(wheels, wheel_type);
        auto wheel_position = wheel_driver.get_wheel_position(dt);

        LOG(TAG, "Measured wheel %s position of: %f!", wheel_type_to_string(wheel_type), wheel_position);
        return wheel_position;
    }

    std::float64_t
    get_wheel_diff_rotation(Wheels& wheels, std::float64_t const dt, std::float64_t const wheel_distance) noexcept
    {
        auto& left_wheel_driver = get_wheel_driver(wheels, WheelType::LEFT);
        auto& right_wheel_driver = get_wheel_driver(wheels, WheelType::RIGHT);

        auto wheel_diff_rotation = (left_wheel_driver.get_wheel_position(dt) / left_wheel_driver.wheel_radius -
                                    right_wheel_driver.get_wheel_position(dt) / right_wheel_driver.wheel_radius) /
                                   wheel_distance;

        LOG(TAG, "Measured wheel diff rotation of: %f!", wheel_diff_rotation);
        return wheel_diff_rotation;
    }

    std::float64_t get_wheel_diff_position(Wheels& wheels, std::float64_t const dt) noexcept
    {
        auto& left_wheel_driver = get_wheel_driver(wheels, WheelType::LEFT);
        auto& right_wheel_driver = get_wheel_driver(wheels, WheelType::RIGHT);

        auto wheel_diff_position = left_wheel_driver.get_wheel_position(dt) - right_wheel_driver.get_wheel_position(dt);

        LOG(TAG, "Measured wheel diff position of: %f!", wheel_diff_position);
        return wheel_diff_position;
    }

    WheelDriver& get_wheel_driver(Wheels& wheels, WheelType const wheel_type) noexcept
    {
        auto* wheel =
            std::ranges::find_if(wheels, [wheel_type](Wheel const& wheel) { return wheel.type == wheel_type; });

        return wheel->driver;
    }

}; // namespace Segway