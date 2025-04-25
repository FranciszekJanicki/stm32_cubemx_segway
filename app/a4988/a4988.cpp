#include "a4988.hpp"
#include "a4988_config.hpp"

namespace a4988 {

    void A4988::initialize(this A4988& self) noexcept
    {
        self.pwm_device.initialize();

        self.set_reset(false);
        self.set_enable(true);
        self.set_sleep(false);
    }

    void A4988::deinitialize(this A4988& self) noexcept
    {
        self.pwm_device.deinitialize();

        self.set_reset(true);
        self.set_enable(false);
        self.set_sleep(true);
    }

    void A4988::start_pwm(this A4988& self) noexcept
    {
        if (!self.is_pwm_started) {
            self.pwm_device.set_compare_half();
            self.is_pwm_started = true;
        }
    }

    void A4988::stop_pwm(this A4988& self) noexcept
    {
        if (self.is_pwm_started) {
            self.pwm_device.set_compare_min();
            self.is_pwm_started = false;
        }
    }

    void A4988::set_frequency(this A4988& self, std::uint32_t const frequency) noexcept
    {
        self.pwm_device.set_frequency(frequency);
    }

    void A4988::set_microstep(this A4988& self, Microstep const microstep) noexcept
    {
        switch (microstep) {
            case Microstep::FULL:
                self.set_full_microstep();
                break;
            case Microstep::HALF:
                self.set_half_microstep();
                break;
            case Microstep::QUARTER:
                self.set_quarter_microstep();
                break;
            case Microstep::EIGHTH:
                self.set_eighth_microstep();
                break;
            case Microstep::SIXTEENTH:
                self.set_sixteenth_microstep();
                break;
            default:
                break;
        }
    }

    void A4988::set_full_microstep(this A4988& self) noexcept
    {
        gpio_write_pin(self.pin_ms1, GPIOState::PIN_RESET);
        gpio_write_pin(self.pin_ms2, GPIOState::PIN_RESET);
        gpio_write_pin(self.pin_ms3, GPIOState::PIN_RESET);
    }

    void A4988::set_half_microstep(this A4988& self) noexcept
    {
        gpio_write_pin(self.pin_ms1, GPIOState::PIN_SET);
        gpio_write_pin(self.pin_ms2, GPIOState::PIN_RESET);
        gpio_write_pin(self.pin_ms3, GPIOState::PIN_RESET);
    }

    void A4988::set_quarter_microstep(this A4988& self) noexcept
    {
        gpio_write_pin(self.pin_ms1, GPIOState::PIN_RESET);
        gpio_write_pin(self.pin_ms2, GPIOState::PIN_SET);
        gpio_write_pin(self.pin_ms3, GPIOState::PIN_RESET);
    }

    void A4988::set_eighth_microstep(this A4988& self) noexcept
    {
        gpio_write_pin(self.pin_ms1, GPIOState::PIN_SET);
        gpio_write_pin(self.pin_ms2, GPIOState::PIN_SET);
        gpio_write_pin(self.pin_ms3, GPIOState::PIN_RESET);
    }

    void A4988::set_sixteenth_microstep(this A4988& self) noexcept
    {
        gpio_write_pin(self.pin_ms1, GPIOState::PIN_RESET);
        gpio_write_pin(self.pin_ms2, GPIOState::PIN_RESET);
        gpio_write_pin(self.pin_ms3, GPIOState::PIN_SET);
    }

    void A4988::set_direction(this A4988& self, Direction const direction) noexcept
    {
        switch (direction) {
            case Direction::FORWARD:
                self.set_forward_direction();
                break;
            case Direction::BACKWARD:
                self.set_backward_direction();
                break;
            case Direction::STOP:
                self.set_stop_direction();
            default:
                break;
        }
    }

    void A4988::set_forward_direction(this A4988& self) noexcept
    {
        gpio_write_pin(self.pin_dir, GPIOState::PIN_RESET);
        self.start_pwm();
    }

    void A4988::set_backward_direction(this A4988& self) noexcept
    {
        gpio_write_pin(self.pin_dir, GPIOState::PIN_SET);
        self.start_pwm();
    }

    void A4988::set_stop_direction(this A4988& self) noexcept
    {
        self.stop_pwm();
    }

    void A4988::set_reset(this A4988 const& self, bool const reset) noexcept
    {
        gpio_write_pin(self.pin_reset, reset ? GPIOState::PIN_RESET : GPIOState::PIN_SET);
    }

    void A4988::set_enable(this A4988 const& self, bool const enable) noexcept
    {
        gpio_write_pin(self.pin_enable, enable ? GPIOState::PIN_RESET : GPIOState::PIN_SET);
    }

    void A4988::set_sleep(this A4988 const& self, bool const sleep) noexcept
    {
        gpio_write_pin(self.pin_sleep, sleep ? GPIOState::PIN_RESET : GPIOState::PIN_SET);
    }

}; // namespace a4988