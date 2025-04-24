#include "wheel.hpp"
#include "log.hpp"

namespace {

    constexpr auto TAG = "Wheel";

};

namespace segway {

    void WheelDriver::start(this WheelDriver& self) noexcept
    {
        self.driver.start();
    }

    void WheelDriver::stop(this WheelDriver& self) noexcept
    {
        self.driver.stop();
    }

    std::float64_t WheelDriver::get_wheel_position(this WheelDriver& self, std::float64_t const dt) noexcept
    {
        return self.driver.get_position(dt) * self.wheel_radius;
    }

    void WheelDriver::set_wheel_position(this WheelDriver& self,
                                         std::float64_t const wheel_position,
                                         std::float64_t const dt) noexcept
    {
        assert(self.wheel_radius);
        self.driver.set_position(wheel_position / self.wheel_radius, dt);
    }

    std::float64_t WheelDriver::get_wheel_speed(this WheelDriver& self, std::float64_t const dt) noexcept
    {
        return self.driver.get_speed(dt) * self.wheel_radius;
    }

    void WheelDriver::set_wheel_speed(this WheelDriver& self,
                                      std::float64_t const wheel_speed,
                                      std::float64_t const dt) noexcept
    {
        assert(self.wheel_radius);
        self.driver.set_speed(wheel_speed / self.wheel_radius, dt);
    }

    char const* wheel_type_to_string(WheelType const wheel_type) noexcept
    {
        switch (wheel_type) {
            case WheelType::LEFT:
                return "LEFT WHEEL";
            case WheelType::RIGHT:
                return "RIGHT WHEEL";
            default:
                return "NONE WHEEL";
        }
    }

}; // namespace segway