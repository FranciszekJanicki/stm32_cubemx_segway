#include "wheel.hpp"

namespace Segway {

    std::float32_t wheel_speed_to_driver_speed(std::float32_t const wheel_speed,
                                               std::float32_t const wheel_radius,
                                               std::float32_t const sampling_time) noexcept
    {
        return wheel_speed * wheel_radius;
    }

    void WheelDriver::set_wheel_speed(this WheelDriver& self,
                                      std::float32_t const wheel_speed,

                                      std::float32_t const sampling_time) noexcept
    {
        auto const driver_speed =
            wheel_speed_to_driver_speed(wheel_speed, self.wheel_radius, sampling_time);
        self.driver.set_speed(driver_speed, sampling_time);
    }

}; // namespace Segway