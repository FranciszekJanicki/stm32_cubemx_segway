#include "wheel.hpp"
#include "log.hpp"
#include <cassert>

namespace {

    constexpr auto TAG = "Wheel";

};

namespace segway {

    void WheelDriver::initialize(this WheelDriver& self) noexcept
    {
        self.driver.initialize();
    }

    void WheelDriver::deinitialize(this WheelDriver& self) noexcept
    {
        self.driver.deinitialize();
    }

    void WheelDriver::start(this WheelDriver& self) noexcept
    {
        self.driver.start();
    }

    void WheelDriver::stop(this WheelDriver& self) noexcept
    {
        self.driver.stop();
    }

    void WheelDriver::update_step_count(this WheelDriver& self) noexcept
    {
        self.driver.update_step_count();
    }

    std::float64_t WheelDriver::get_wheel_position(this WheelDriver& self,
                                                   std::float64_t const dt) noexcept
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

    std::float64_t WheelDriver::get_wheel_speed(this WheelDriver& self,
                                                std::float64_t const dt) noexcept
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
            case WheelType::WHEEL_LEFT:
                return "LEFT WHEEL";
            case WheelType::WHEEL_RIGHT:
                return "RIGHT WHEEL";
        }
    }

}; // namespace segway