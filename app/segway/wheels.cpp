#include "wheels.hpp"

namespace Segway {
    void Wheels::update_wheel_step_count(this Wheels& self, WheelType const wheel_type) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        wheel_driver.driver.update_step_count();
    }

    void Wheels::set_wheel_speed(this Wheels& self,
                                 WheelType const wheel_type,
                                 std::float32_t const wheel_speed,
                                 std::float32_t const dt) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        wheel_driver.set_wheel_speed(wheel_speed, dt);
    }

    std::float32_t
    Wheels::get_wheel_speed(this Wheels& self, WheelType const wheel_type, std::float32_t const dt) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        return wheel_driver.get_wheel_speed(dt);
    }

    void Wheels::set_wheel_position(this Wheels& self,
                                    WheelType const wheel_type,
                                    std::float32_t const wheel_position,
                                    std::float32_t const dt) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        wheel_driver.set_wheel_position(wheel_position, dt);
    }

    std::float32_t
    Wheels::get_wheel_position(this Wheels& self, WheelType const wheel_type, std::float32_t const dt) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        return wheel_driver.get_wheel_position(dt);
    }

    std::float32_t Wheels::get_wheel_diff_rotation(this Wheels& self, std::float32_t const dt) noexcept
    {
        return (self.get_wheel_position(WheelType::RIGHT, dt) - self.get_wheel_position(WheelType::LEFT, dt)) /
               self.wheel_base_len;
    }

    std::float32_t Wheels::get_wheel_diff_position(this Wheels& self, std::float32_t const dt) noexcept
    {
        return self.get_wheel_position(WheelType::RIGHT, dt) - self.get_wheel_position(WheelType::LEFT, dt);
    }

    WheelDriver& Wheels::get_wheel_driver(this Wheels& self, WheelType const wheel_type) noexcept
    {
        auto* wheel =
            std::ranges::find_if(self.wheels, [wheel_type](Wheel const& wheel) { return wheel.type == wheel_type; });

        return wheel->driver;
    }

}; // namespace Segway