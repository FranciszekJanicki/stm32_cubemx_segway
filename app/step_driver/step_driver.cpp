#include "step_driver.hpp"

namespace StepDriver {

    Microstep speed_to_microstep(std::float64_t const speed, std::float64_t const step_change) noexcept
    {
        return Microstep::FULL;
    }

    Direction speed_to_direction(std::float64_t const speed) noexcept
    {
        return speed > 0.0F ? Direction::FORWARD : Direction::BACKWARD;
    }

    std::float64_t frequency_to_speed(std::uint16_t const frequency, std::float64_t const step_change) noexcept
    {
        return static_cast<std::float64_t>(std::abs(frequency * step_change));
    }

    std::uint16_t speed_to_frequency(std::float64_t const speed, std::float64_t const step_change) noexcept
    {
        return static_cast<std::uint16_t>(std::abs(speed / step_change));
    }

    void StepDriver::update_step_count(this StepDriver& self) noexcept
    {
        auto const counter_period = self.driver.pwm_device_.get_period();

        if (self.direction == Direction::BACKWARD) {
            self.step_count = std::max(0LL, self.step_count - 1LL);
        } else if (self.direction == Direction::FORWARD) {
            self.step_count = std::max(0LL, self.step_count + 1LL);
        }
    }

    void
    StepDriver::set_position(this StepDriver& self, std::float64_t const position, std::float64_t const dt) noexcept
    {
        auto const speed = Utility::differentiate(position, std::exchange(self.prev_position, position), dt);

        self.set_speed(speed, dt);
    }

    void StepDriver::set_speed(this StepDriver& self, std::float64_t const speed, std::float64_t const dt) noexcept
    {
        auto const error_speed = speed - self.get_speed(dt);
        auto control_speed = speed;
        // self.regulator(error_speed, dt);

        if (std::abs(control_speed) < MIN_SPEED) {
            //  self.stop();
            //  self.set_control_speed(0.0F64);
        } else {
            self.start();
            self.set_control_speed(control_speed);
        }
    }

    void StepDriver::set_acceleration(this StepDriver& self,
                                      std::float64_t const acceleration,
                                      std::float64_t const dt) noexcept
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

    std::float64_t StepDriver::step_change(this StepDriver& self) noexcept
    {
        return (360.0F / static_cast<std::float64_t>(self.steps_per_360)) * microstep_to_fraction(self.microstep);
    }

    void StepDriver::set_control_speed(this StepDriver& self, std::float64_t const control_speed) noexcept
    {
        auto speed = std::clamp(control_speed, -MAX_SPEED, MAX_SPEED);

        auto step_change = self.step_change();

        self.set_direction(speed_to_direction(speed));
        self.set_microstep(speed_to_microstep(speed, step_change));
        self.set_frequency(speed_to_frequency(speed, step_change));
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

    std::float64_t StepDriver::get_position(this StepDriver& self, std::float64_t const dt) noexcept
    {
        return std::fmodf(self.step_change() * static_cast<std::float64_t>(self.step_count), 360.0F);
    }

    std::float64_t StepDriver::get_speed(this StepDriver& self, std::float64_t const dt) noexcept
    {
        auto const position = self.get_position(dt);

        if (!self.is_initialized) {
            self.prev_position = position;
            self.is_initialized = true;
        }

        return Utility::differentiate(position, std::exchange(self.prev_position, position), dt);
    }

    std::float64_t StepDriver::get_acceleration(this StepDriver& self, std::float64_t const dt) noexcept
    {
        auto const speed = self.get_speed(dt);

        return Utility::differentiate(speed, std::exchange(self.prev_speed, speed), dt);
    }
}; // namespace StepDriver