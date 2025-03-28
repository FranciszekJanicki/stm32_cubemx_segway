#include "step_driver.hpp"
#include <cinttypes>

namespace StepDriver {

    Microstep speed_to_microstep(float const speed, float const step_change) noexcept
    {
        return Microstep::FULL;
    }

    Direction speed_to_direction(float const speed) noexcept
    {
        return speed > 0.0F ? Direction::FORWARD : Direction::BACKWARD;
    }

    float frequency_to_speed(std::uint16_t const frequency, float const step_change) noexcept
    {
        return static_cast<float>(std::abs(frequency * step_change));
    }

    std::uint16_t speed_to_frequency(float const speed, float const step_change) noexcept
    {
        return static_cast<std::uint16_t>(std::abs(speed / step_change));
    }

    float get_processed_speed(float const speed,
                              float const prev_speed,
                              float const step_change,
                              float const sampling_time,
                              float const max_speed,
                              float const max_acceleration) noexcept
    {
        auto const clamped_speed = std::clamp(speed, -max_speed, max_speed);
        auto const acceleration = Utility::differentiate(clamped_speed, prev_speed, sampling_time);
        auto const clamped_acceleration = std::clamp(acceleration, -max_acceleration, max_acceleration);

        return clamped_acceleration * sampling_time + prev_speed;
    }

    void StepDriver::update_step_count() noexcept
    {
        auto const counter_period = this->driver.pwm_device_.get_counter_period();

        if (this->direction == Direction::BACKWARD) {
            this->step_count = (this->step_count - 1U) % counter_period;
        } else if (this->direction == Direction::FORWARD) {
            this->step_count = (this->step_count + 1U) % counter_period;
        }
    }

    void StepDriver::set_position(float const position, float const sampling_time) noexcept
    {
        auto const speed =
            Utility::differentiate(position, std::exchange(this->prev_position, position), sampling_time);

        this->set_speed(speed, sampling_time);
    }

    void StepDriver::set_speed(float const speed, float const sampling_time) noexcept
    {
        auto const error_speed = speed - this->get_speed(sampling_time);
        auto control_speed = speed;
        // this->regulator(error_speed, sampling_time);

        auto static stopped = false;

        if (std::abs(control_speed) < MIN_SPEED) {
            this->stop_pwm();
        } else {
            if (this->stopped) {
                this->start_pwm();
            }
            this->set_control_speed(speed);
        }
    }

    void StepDriver::set_acceleration(float const acceleration, float const sampling_time) noexcept
    {
        auto const speed =
            Utility::integrate(acceleration, std::exchange(this->prev_acceleration, acceleration), sampling_time);

        this->set_speed(speed, sampling_time);
    }

    void StepDriver::start_pwm() noexcept
    {
        this->driver.pwm_device_.start_it();
        this->stopped = false;
    }

    void StepDriver::stop_pwm() noexcept
    {
        this->driver.pwm_device_.stop_it();
        this->stopped = true;
    }

    float StepDriver::step_change() const noexcept
    {
        return (360.0F / static_cast<float>(this->steps_per_360)) * microstep_to_fraction(this->microstep);
    }

    void StepDriver::set_control_speed(float const control_speed) noexcept
    {
        this->set_direction(speed_to_direction(control_speed));
        this->set_microstep(speed_to_microstep(control_speed, this->step_change()));
        this->set_frequency(speed_to_frequency(control_speed, this->step_change()));
    }

    void StepDriver::set_microstep(Microstep const microstep) noexcept
    {
        this->microstep = microstep;
        this->driver.set_microstep(microstep);
    }

    void StepDriver::set_direction(Direction const direction) noexcept
    {
        this->direction = direction;
        this->driver.set_direction(direction);
    }

    void StepDriver::set_frequency(std::uint16_t const frequency) noexcept
    {
        this->frequency = frequency;
        this->driver.set_frequency(frequency);
    }

    float StepDriver::get_position(float const sampling_time) noexcept
    {
        return std::fmodf(this->step_change() * this->step_count, 360.0F);
    }

    float StepDriver::get_speed(float const sampling_time) noexcept
    {
        auto const position = this->get_position(sampling_time);

        return Utility::differentiate(position, std::exchange(this->prev_position, position), sampling_time);
    }

    float StepDriver::get_acceleration(float const sampling_time) noexcept
    {
        auto const speed = this->get_speed(sampling_time);

        return Utility::differentiate(speed, std::exchange(this->prev_speed, speed), sampling_time);
    }

}; // namespace StepDriver