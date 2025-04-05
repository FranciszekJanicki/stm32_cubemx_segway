#include "segway.hpp"

namespace Segway {

    void Segway::update_step_count(Channel const channel) noexcept
    {
        auto& driver = this->get_driver(channel);
        driver.update_step_count();
    }

    void
    Segway::set_speed(Channel const channel, std::float32_t const speed, std::float32_t const sampling_time) noexcept
    {
        auto& driver = this->get_driver(channel);
        driver.set_speed(speed, sampling_time);
    }

    void Segway::operator()(std::float32_t const angle, std::float32_t const sampling_time) noexcept
    {
        this->set_angle(angle, sampling_time);
    }

    void Segway::set_angle(std::float32_t const angle, std::float32_t const sampling_time) noexcept
    {
        auto const measured_angle = std::visit([](auto& imu) { return imu.get_roll().value_or(0.0F); }, this->imu);
        std::printf("roll: %f\n\r", measured_angle);

        auto const error_angle = angle - measured_angle;
        auto const error_angular_speed = this->angle_to_angular_speed(error_angle, sampling_time);
        auto const control_speed = this->regulator(error_angle /*error_angular_speed*/, sampling_time);

        this->set_speed(Channel::CHANNEL_1, control_speed, sampling_time);
        this->set_speed(Channel::CHANNEL_2, control_speed, sampling_time);
    }

    std::float32_t Segway::angle_to_angular_speed(std::float32_t const angle,
                                                  std::float32_t const sampling_time) noexcept
    {
        return Utility::differentiate(angle, std::exchange(this->prev_control_speed, angle), sampling_time);
    }

    Driver& Segway::get_driver(Channel const channel) noexcept
    {
        if (auto it = std::ranges::find_if(
                this->driver_channels,
                [channel](DriverChannel const& driver_channel) { return driver_channel.channel == channel; });
            it != this->driver_channels.end()) {
            return it->driver;
        } else {
            std::unreachable();
        }
    }

}; // namespace Segway