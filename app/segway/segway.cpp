#include "segway.hpp"

namespace Segway {

    void Segway::update_step_count(this Segway& self, Channel const channel) noexcept
    {
        auto& driver = self.get_driver(channel);
        driver.update_step_count();
    }

    void Segway::set_speed(this Segway& self,
                           Channel const channel,
                           std::float32_t const speed,
                           std::float32_t const sampling_time) noexcept
    {
        auto& driver = self.get_driver(channel);
        driver.set_speed(speed, sampling_time);
    }

    void Segway::operator()(this Segway& self, std::float32_t const angle, std::float32_t const sampling_time) noexcept
    {
        self.set_angle(angle, sampling_time);
    }

    void Segway::set_angle(this Segway& self, std::float32_t const angle, std::float32_t const sampling_time) noexcept
    {
        auto const measured_angle = std::visit([](auto& imu) { return imu.get_roll().value_or(0.0F); }, self.imu);
        std::printf("roll: %f\n\r", measured_angle);

        auto const error_angle = angle - measured_angle;
        auto const error_angular_speed = self.angle_to_angular_speed(error_angle, sampling_time);
        auto const control_speed = self.regulator(error_angle /*error_angular_speed*/, sampling_time);

        self.set_speed(Channel::CHANNEL_1, control_speed, sampling_time);
        self.set_speed(Channel::CHANNEL_2, control_speed, sampling_time);
    }

    std::float32_t Segway::angle_to_angular_speed(this Segway& self,
                                                  std::float32_t const angle,
                                                  std::float32_t const sampling_time) noexcept
    {
        return Utility::differentiate(angle, std::exchange(self.prev_control_speed, angle), sampling_time);
    }

    Driver& Segway::get_driver(this Segway& self, Channel const channel) noexcept
    {
        if (auto it = std::ranges::find_if(
                self.driver_channels,
                [channel](DriverChannel const& driver_channel) { return driver_channel.channel == channel; });
            it != self.driver_channels.end()) {
            return it->driver;
        } else {
            std::unreachable();
        }
    }

}; // namespace Segway