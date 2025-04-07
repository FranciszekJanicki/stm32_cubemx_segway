#include "a4988.hpp"
#include "a4988_config.hpp"

namespace A4988 {

    A4988::A4988(PWMDevice&& pwm_device,
                 GPIO const pin_ms1,
                 GPIO const pin_ms2,
                 GPIO const pin_ms3,
                 GPIO const pin_reset,
                 GPIO const pin_sleep,
                 GPIO const pin_dir,
                 GPIO const pin_enable) noexcept :
        pwm_device_{std::forward<PWMDevice>(pwm_device)},
        pin_ms1_{pin_ms1},
        pin_ms2_{pin_ms2},
        pin_ms3_{pin_ms3},
        pin_reset_{pin_reset},
        pin_sleep_{pin_sleep},
        pin_dir_{pin_dir},
        pin_enable_{pin_enable}
    {
        this->initialize();
    }

    A4988::~A4988() noexcept
    {
        this->deinitialize();
    }

    void A4988::start(this A4988 const& self) noexcept
    {
        self.pwm_device_.set_compare_half();
    }

    void A4988::stop(this A4988 const& self) noexcept
    {
        self.pwm_device_.set_compare_min();
    }

    void A4988::set_frequency(this A4988 const& self, std::uint32_t const frequency) noexcept
    {
        self.pwm_device_.set_frequency(frequency);
    }

    void A4988::set_microstep(this A4988 const& self, Microstep const microstep) noexcept
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

    void A4988::set_full_microstep(this A4988 const& self) noexcept
    {
        gpio_write_pin(self.pin_ms1_, GPIO_PIN_RESET);
        gpio_write_pin(self.pin_ms2_, GPIO_PIN_RESET);
        gpio_write_pin(self.pin_ms3_, GPIO_PIN_RESET);
    }

    void A4988::set_half_microstep(this A4988 const& self) noexcept
    {
        gpio_write_pin(self.pin_ms1_, GPIO_PIN_SET);
        gpio_write_pin(self.pin_ms2_, GPIO_PIN_RESET);
        gpio_write_pin(self.pin_ms3_, GPIO_PIN_RESET);
    }

    void A4988::set_quarter_microstep(this A4988 const& self) noexcept
    {
        gpio_write_pin(self.pin_ms1_, GPIO_PIN_RESET);
        gpio_write_pin(self.pin_ms2_, GPIO_PIN_SET);
        gpio_write_pin(self.pin_ms3_, GPIO_PIN_RESET);
    }

    void A4988::set_eighth_microstep(this A4988 const& self) noexcept
    {
        gpio_write_pin(self.pin_ms1_, GPIO_PIN_SET);
        gpio_write_pin(self.pin_ms2_, GPIO_PIN_SET);
        gpio_write_pin(self.pin_ms3_, GPIO_PIN_RESET);
    }

    void A4988::set_sixteenth_microstep(this A4988 const& self) noexcept
    {
        gpio_write_pin(self.pin_ms1_, GPIO_PIN_RESET);
        gpio_write_pin(self.pin_ms2_, GPIO_PIN_RESET);
        gpio_write_pin(self.pin_ms3_, GPIO_PIN_SET);
    }

    void A4988::set_direction(this A4988 const& self, Direction const direction) noexcept
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

    void A4988::set_forward_direction(this A4988 const& self) noexcept
    {
        gpio_write_pin(self.pin_dir_, GPIO_PIN_RESET);
        self.start();
    }

    void A4988::set_backward_direction(this A4988 const& self) noexcept
    {
        gpio_write_pin(self.pin_dir_, GPIO_PIN_SET);
        self.start();
    }

    void A4988::set_stop_direction(this A4988 const& self) noexcept
    {
        self.stop();
    }

    void A4988::set_reset(this A4988 const& self, bool const reset) noexcept
    {
        gpio_write_pin(self.pin_reset_, reset ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    void A4988::set_enable(this A4988 const& self, bool const enable) noexcept
    {
        gpio_write_pin(self.pin_enable_, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    void A4988::set_sleep(this A4988 const& self, bool const sleep) noexcept
    {
        gpio_write_pin(self.pin_sleep_, sleep ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    void A4988::initialize(this A4988 const& self) noexcept
    {
        self.set_reset(false);
        self.set_enable(true);
        self.set_sleep(false);
        self.stop();
    }

    void A4988::deinitialize(this A4988 const& self) noexcept
    {
        self.set_reset(true);
        self.set_enable(false);
        self.set_sleep(true);
        self.stop();
    }

}; // namespace A4988