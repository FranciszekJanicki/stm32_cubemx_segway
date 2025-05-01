#include "step_driver.hpp"
#include "utility.hpp"

namespace step_driver {

    Microstep speed_to_microstep(std::float64_t const speed,
                                 std::float64_t const step_change) noexcept
    {
        return Microstep::FULL;
    }

    Direction speed_to_direction(std::float64_t const speed) noexcept
    {
        return speed > 0.0F ? Direction::FORWARD : Direction::BACKWARD;
    }

    std::uint32_t speed_to_frequency(std::float64_t const speed,
                                     std::float64_t const step_change) noexcept
    {
        return static_cast<std::uint16_t>(std::abs(speed / step_change));
    }

    void StepDriver::initialize(this StepDriver& self) noexcept
    {
        if (!self.is_initialized) {
            self.driver.initialize();
            self.is_initialized = true;
        }
    }

    void StepDriver::deinitialize(this StepDriver& self) noexcept
    {
        if (!self.is_initialized) {
            self.driver.deinitialize();
            self.is_initialized = true;
        }
    }

    void StepDriver::start(this StepDriver& self) noexcept
    {
        if (self.is_stopped) {
            self.driver.start_pulses();
            self.is_stopped = false;
        }
    }

    void StepDriver::stop(this StepDriver& self) noexcept
    {
        if (!self.is_stopped) {
            self.driver.stop_pulses();
            self.is_stopped = true;
        }
    }

    void StepDriver::update_step_count(this StepDriver& self) noexcept
    {
        if (self.direction == Direction::BACKWARD) {
            self.step_count = std::max(0LL, self.step_count - 1LL);
        } else if (self.direction == Direction::FORWARD) {
            self.step_count = std::max(0LL, self.step_count + 1LL);
        }
    }

    void StepDriver::set_position(this StepDriver& self,
                                  std::float64_t const position,
                                  std::float64_t const dt) noexcept
    {
        auto const speed =
            utility::differentiate(position, std::exchange(self.prev_position, position), dt);

        self.set_speed(speed, dt);
    }

    void StepDriver::set_speed(this StepDriver& self,
                               std::float64_t const speed,
                               std::float64_t const dt) noexcept
    {
        // auto const error_speed = speed - self.get_speed(dt);
        auto control_speed = speed; // self.regulator(error_speed, dt);

        self.set_control_speed(control_speed);
    }

    void StepDriver::set_acceleration(this StepDriver& self,
                                      std::float64_t const acceleration,
                                      std::float64_t const dt) noexcept
    {
        auto const speed = utility::integrate(acceleration,
                                              std::exchange(self.prev_acceleration, acceleration),
                                              dt);

        self.set_speed(speed, dt);
    }

    std::float64_t StepDriver::step_change(this StepDriver& self) noexcept
    {
        return (360.0F64 / static_cast<std::float64_t>(self.steps_per_360)) *
               microstep_to_fraction(self.microstep);
    }

    void StepDriver::set_control_speed(this StepDriver& self,
                                       std::float64_t const control_speed) noexcept
    {
        if (self.should_stop(control_speed)) {
            self.stop();
            return;
        } else if (self.should_start(control_speed)) {
            self.start();
        }

        auto clamped_speed = std::clamp(control_speed, -MAX_SPEED, MAX_SPEED);
        auto step_change = self.step_change();

        self.set_direction(speed_to_direction(clamped_speed));
        self.set_microstep(speed_to_microstep(clamped_speed, step_change));
        self.set_frequency(speed_to_frequency(clamped_speed, step_change));
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

    void StepDriver::set_frequency(this StepDriver& self, std::uint32_t const frequency) noexcept
    {
        self.frequency = frequency;
        self.driver.set_frequency(frequency);
    }

    std::float64_t StepDriver::get_position(this StepDriver& self, std::float64_t const dt) noexcept
    {
        return std::fmodf(self.step_change() * static_cast<std::float64_t>(self.step_count),
                          360.0F);
    }

    std::float64_t StepDriver::get_speed(this StepDriver& self, std::float64_t const dt) noexcept
    {
        auto const position = self.get_position(dt);

        return utility::differentiate(position, std::exchange(self.prev_position, position), dt);
    }

    std::float64_t StepDriver::get_acceleration(this StepDriver& self,
                                                std::float64_t const dt) noexcept
    {
        auto const speed = self.get_speed(dt);

        return utility::differentiate(speed, std::exchange(self.prev_speed, speed), dt);
    }

    bool StepDriver::should_start(this StepDriver& self,
                                  std::float64_t const control_speed) noexcept
    {
        return (std::abs(control_speed) > MIN_SPEED && std::abs(control_speed) < MAX_SPEED) &&
               self.is_stopped;
    }

    bool StepDriver::should_stop(this StepDriver& self, std::float64_t const control_speed) noexcept
    {
        return (std::abs(control_speed) < MIN_SPEED || std::abs(control_speed) > MAX_SPEED) &&
               !self.is_stopped;
    }
}; // namespace step_driver