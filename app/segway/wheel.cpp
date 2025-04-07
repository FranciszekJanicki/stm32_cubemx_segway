#include "wheel.hpp"

namespace Segway {

    void WheelDriver::set_wheel_speed(this WheelDriver& self,
                                      std::float32_t const wheel_speed,
                                      std::float32_t const sampling_time) noexcept
    {
        self.driver.set_speed(wheel_speed / self.wheel_radius, sampling_time);
    }

    std::float32_t WheelDriver::get_wheel_position(this WheelDriver& self,
                                                   std::float32_t const sampling_time) noexcept
    {
        return self.driver.get_position(sampling_time) * self.wheel_radius;
    }

    std::float32_t WheelDriver::get_wheel_speed(this WheelDriver& self,
                                                std::float32_t const sampling_time) noexcept
    {
        return self.driver.get_speed(sampling_time) * self.wheel_radius;
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

}; // namespace Segway