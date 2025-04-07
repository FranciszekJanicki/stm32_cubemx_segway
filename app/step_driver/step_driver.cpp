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

    void StepDriver::update_step_count(this StepDriver& self) noexcept
    {
        auto const counter_period = self.driver.pwm_device_.get_period();

        if (self.direction == Direction::BACKWARD) {
            self.step_count = (self.step_count - 1U) % counter_period;
        } else if (self.direction == Direction::FORWARD) {
            self.step_count = (self.step_count + 1U) % counter_period;
        }
    }

    void
    StepDriver::set_position(this StepDriver& self, std::float32_t const position, std::float32_t const dt) noexcept
    {
        auto const speed = Utility::differentiate(position, std::exchange(self.prev_position, position), dt);

        self.set_speed(speed, dt);
    }

    void StepDriver::set_speed(this StepDriver& self, std::float32_t const speed, std::float32_t const dt) noexcept
    {
        auto const error_speed = speed - self.get_speed(dt);
        auto control_speed = speed;
        // self.regulator(error_speed, dt);

        if (std::abs(control_speed) < 10.0F) {
            self.stop();
        } else {
            if (self.is_stopped) {
                self.start();
            }
            self.set_control_speed(speed);
        }
    }

    void StepDriver::set_acceleration(this StepDriver& self,
                                      std::float32_t const acceleration,
                                      std::float32_t const dt) noexcept
    {
        auto const speed = Utility::integrate(acceleration, std::exchange(self.prev_acceleration, acceleration), dt);

        self.set_speed(speed, dt);
    }

    void StepDriver::start(this StepDriver& self) noexcept
    {
        self.driver.start();
        self.is_stopped = false;
    }

    void StepDriver::stop(this StepDriver& self) noexcept
    {
        self.driver.stop();
        self.is_stopped = true;
    }

    std::float32_t StepDriver::step_change(this StepDriver& self) noexcept
    {
        return (360.0F / static_cast<std::float32_t>(self.steps_per_360)) * microstep_to_fraction(self.microstep);
    }

    void StepDriver::set_control_speed(this StepDriver& self, std::float32_t const control_speed) noexcept
    {
        self.set_direction(speed_to_direction(control_speed));
        self.set_microstep(speed_to_microstep(control_speed, self.step_change()));
        self.set_frequency(speed_to_frequency(control_speed, self.step_change()));
    }

    void StepDriver::set_microstep(this StepDriver& self, Microstep const microstep) noexcept
    {
        self.microstep = microstep;
        self.driver.set_microstep(microstep);
    }

    void StepDriver::set_direction(this StepDriver& self, Direction const direction) noexcept
    {
        self.direction = direction;
        self.driver.set_direction(direction);
    }

    void StepDriver::set_frequency(this StepDriver& self, std::uint16_t const frequency) noexcept
    {
        self.frequency = frequency;
        self.driver.set_frequency(frequency);
    }

    std::float32_t StepDriver::get_position(this StepDriver& self, std::float32_t const dt) noexcept
    {
        return std::fmodf(self.step_change() * static_cast<std::float32_t>(self.step_count), 360.0F);
    }

    std::float32_t StepDriver::get_speed(this StepDriver& self, std::float32_t const dt) noexcept
    {
        auto const position = self.get_position(dt);

        return Utility::differentiate(position, std::exchange(self.prev_position, position), dt);
    }

    std::float32_t StepDriver::get_acceleration(this StepDriver& self, std::float32_t const dt) noexcept
    {
        auto const speed = self.get_speed(dt);

        return Utility::differentiate(speed, std::exchange(self.prev_speed, speed), dt);
    }

}; // namespace StepDriver