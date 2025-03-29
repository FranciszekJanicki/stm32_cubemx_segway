#include "segway.hpp"

namespace Segway {

    void Segway::update_step_count() noexcept
    {
        for (auto& driver : this->drivers) {
            driver.update_step_count();
        }
    }

    void Segway::operator()(std::float32_t const angle, std::float32_t const sampling_time) noexcept
    {
        this->set_angle(angle, sampling_time);
    }

    void Segway::set_angle(std::float32_t const angle, std::float32_t const sampling_time) noexcept
    {
        auto const error_angle = angle - this->imu.get_roll().value_or(0.0F);
        //  auto const error_speed = this->angle_to_angular_speed(error_angle, sampling_time);
        auto const control_speed = this->regulator(error_angle, sampling_time);

        for (auto& driver : this->drivers) {
            driver.set_speed(control_speed, sampling_time);
        }
    }

    std::float32_t Segway::angle_to_angular_speed(std::float32_t const angle,
                                                  std::float32_t const sampling_time) noexcept
    {
        return Utility::differentiate(angle, std::exchange(this->prev_control_speed, angle), sampling_time);
    }

}; // namespace Segway