#include "a4988.hpp"
#include "a4988_config.hpp"

namespace a4988 {

    void A4988::initialize(this A4988& self) noexcept
    {
        if (!self.is_initialized) {
            self.pulse_init();
            self.gpio_init();
            self.set_reset(false);
            self.set_enable(true);
            self.set_sleep(false);
            self.is_initialized = true;
        }
    }

    void A4988::deinitialize(this A4988& self) noexcept
    {
        if (self.is_initialized) {
            self.set_reset(true);
            self.set_enable(false);
            self.set_sleep(true);
            self.pulse_deinit();
            self.gpio_deinit();
            self.is_initialized = false;
        }
    }

    void A4988::start_pulses(this A4988& self) noexcept
    {
        if (!self.has_pulses_started) {
            self.pulse_start();
            self.has_pulses_started = true;
        }
    }

    void A4988::stop_pulses(this A4988& self) noexcept
    {
        if (self.has_pulses_started) {
            self.pulse_stop();
            self.has_pulses_started = false;
        }
    }

    void A4988::set_frequency(this A4988& self, std::uint32_t const frequency) noexcept
    {
        self.pulse_set_freq(frequency);
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
        self.gpio_write_pin(self.config.pin_ms1, false);
        self.gpio_write_pin(self.config.pin_ms2, false);
        self.gpio_write_pin(self.config.pin_ms3, false);
    }

    void A4988::set_half_microstep(this A4988& self) noexcept
    {
        self.gpio_write_pin(self.config.pin_ms1, true);
        self.gpio_write_pin(self.config.pin_ms2, false);
        self.gpio_write_pin(self.config.pin_ms3, false);
    }

    void A4988::set_quarter_microstep(this A4988& self) noexcept
    {
        self.gpio_write_pin(self.config.pin_ms1, true);
        self.gpio_write_pin(self.config.pin_ms2, true);
        self.gpio_write_pin(self.config.pin_ms3, false);
    }

    void A4988::set_eighth_microstep(this A4988& self) noexcept
    {
        self.gpio_write_pin(self.config.pin_ms1, true);
        self.gpio_write_pin(self.config.pin_ms2, true);
        self.gpio_write_pin(self.config.pin_ms3, false);
    }

    void A4988::set_sixteenth_microstep(this A4988& self) noexcept
    {
        self.gpio_write_pin(self.config.pin_ms1, false);
        self.gpio_write_pin(self.config.pin_ms2, false);
        self.gpio_write_pin(self.config.pin_ms3, true);
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
        self.gpio_write_pin(self.config.pin_dir, false);
        self.start_pulses();
    }

    void A4988::set_backward_direction(this A4988& self) noexcept
    {
        self.gpio_write_pin(self.config.pin_dir, true);
        self.start_pulses();
    }

    void A4988::set_stop_direction(this A4988& self) noexcept
    {
        self.stop_pulses();
    }

    void A4988::set_reset(this A4988 const& self, bool const reset) noexcept
    {
        self.gpio_write_pin(self.config.pin_enable, !reset);
    }

    void A4988::set_enable(this A4988 const& self, bool const enable) noexcept
    {
        self.gpio_write_pin(self.config.pin_enable, !enable);
    }

    void A4988::set_sleep(this A4988 const& self, bool const sleep) noexcept
    {
        self.gpio_write_pin(self.config.pin_sleep, !sleep);
    }

    void A4988::pulse_init(this A4988 const& self) noexcept
    {
        if (self.interface.pulse_init) {
            self.interface.pulse_init(self.interface.pulse_user);
        }
    }

    void A4988::pulse_deinit(this A4988 const& self) noexcept
    {
        if (self.interface.pulse_deinit) {
            self.interface.pulse_deinit(self.interface.pulse_user);
        }
    }

    void A4988::pulse_start(this A4988 const& self) noexcept
    {
        if (self.interface.pulse_start) {
            self.interface.pulse_start(self.interface.pulse_user);
        }
    }

    void A4988::pulse_stop(this A4988 const& self) noexcept
    {
        if (self.interface.pulse_stop) {
            self.interface.pulse_stop(self.interface.pulse_user);
        }
    }

    void A4988::pulse_set_freq(this A4988 const& self, std::uint32_t const freq) noexcept
    {
        if (self.interface.pulse_set_freq) {
            self.interface.pulse_set_freq(self.interface.pulse_user, freq);
        }
    }

    void A4988::gpio_init(this A4988 const& self) noexcept
    {
        if (self.interface.gpio_init) {
            self.interface.gpio_init(self.interface.gpio_user);
        }
    }

    void A4988::gpio_deinit(this A4988 const& self) noexcept
    {
        if (self.interface.gpio_deinit) {
            self.interface.gpio_deinit(self.interface.gpio_user);
        }
    }

    void A4988::gpio_write_pin(this A4988 const& self,
                               std::int32_t const pin,
                               bool const state) noexcept
    {
        if (self.interface.gpio_write_pin) {
            self.interface.gpio_write_pin(self.interface.gpio_user, pin, state);
        }
    }

}; // namespace a4988