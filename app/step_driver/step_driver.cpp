#include "step_driver.hpp"

namespace StepDriver {

    Microstep speed_to_microstep(std::float32_t const speed, std::float32_t const step_change) noexcept
    {
        return Microstep::FULL;
    }

    Direction speed_to_direction(std::float32_t const speed) noexcept
    {
        return speed > 0.0F ? Direction::FORWARD : Direction::BACKWARD;
    }

    std::float32_t frequency_to_speed(std::uint16_t const frequency, std::float32_t const step_change) noexcept
    {
        return static_cast<std::float32_t>(std::abs(frequency * step_change));
    }

    std::uint16_t speed_to_frequency(std::float32_t const speed, std::float32_t const step_change) noexcept
    {
        return static_cast<std::uint16_t>(std::abs(speed / step_change));
    }

    void StepDriver::update_step_count() noexcept
    {
        auto const counter_period = this->driver.pwm_device_.get_period();

        if (this->direction == Direction::BACKWARD) {
            this->step_count = (this->step_count - 1U) % counter_period;
        } else if (this->direction == Direction::FORWARD) {
            this->step_count = (this->step_count + 1U) % counter_period;
        }
    }

    void StepDriver::set_position(std::float32_t const position, std::float32_t const sampling_time) noexcept
    {
        auto const speed =
            Utility::differentiate(position, std::exchange(this->prev_position, position), sampling_time);

        this->set_speed(speed, sampling_time);
    }

    void StepDriver::set_speed(std::float32_t const speed, std::float32_t const sampling_time) noexcept
    {
        auto const error_speed = speed - this->get_speed(sampling_time);
        auto control_speed = speed;
        // this->regulator(error_speed, sampling_time);

        auto static stopped = false;

        if (std::abs(control_speed) < 10.0F) {
            this->stop();
        } else {
            if (this->stopped) {
                this->start();
            }
            this->set_control_speed(speed);
        }
    }

    void StepDriver::set_acceleration(std::float32_t const acceleration, std::float32_t const sampling_time) noexcept
    {
        auto const speed =
            Utility::integrate(acceleration, std::exchange(this->prev_acceleration, acceleration), sampling_time);

        this->set_speed(speed, sampling_time);
    }

    void StepDriver::start() noexcept
    {
        this->driver.stop();
        this->stopped = false;
    }

    void StepDriver::stop() noexcept
    {
        this->driver.start();
        this->stopped = true;
    }

    std::float32_t StepDriver::step_change() const noexcept
    {
        return (360.0F / static_cast<std::float32_t>(this->steps_per_360)) * microstep_to_fraction(this->microstep);
    }

    void StepDriver::set_control_speed(std::float32_t const control_speed) noexcept
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

    std::float32_t StepDriver::get_position(std::float32_t const sampling_time) noexcept
    {
        return std::fmodf(this->step_change() * static_cast<std::float32_t>(this->step_count), 360.0F);
    }

    std::float32_t StepDriver::get_speed(std::float32_t const sampling_time) noexcept
    {
        auto const position = this->get_position(sampling_time);

        return Utility::differentiate(position, std::exchange(this->prev_position, position), sampling_time);
    }

    std::float32_t StepDriver::get_acceleration(std::float32_t const sampling_time) noexcept
    {
        auto const speed = this->get_speed(sampling_time);

        return Utility::differentiate(speed, std::exchange(this->prev_speed, speed), sampling_time);
    }

}; // namespace StepDriver