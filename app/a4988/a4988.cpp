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

    void A4988::set_frequency(std::uint32_t const frequency) noexcept
    {
        this->pwm_device_.set_frequency(frequency);
        this->set_step();
    }

    void A4988::set_microstep(Microstep const microstep) const noexcept
    {
        switch (microstep) {
            case Microstep::FULL:
                this->set_full_microstep();
                break;
            case Microstep::HALF:
                this->set_half_microstep();
                break;
            case Microstep::QUARTER:
                this->set_quarter_microstep();
                break;
            case Microstep::EIGHTH:
                this->set_eighth_microstep();
                break;
            case Microstep::SIXTEENTH:
                this->set_sixteenth_microstep();
                break;
            default:
                break;
        }
    }

    void A4988::set_full_microstep() const noexcept
    {
        if (this->initialized_) {
            gpio_write_pin(this->pin_ms1_, GPIO_PIN_RESET);
            gpio_write_pin(this->pin_ms2_, GPIO_PIN_RESET);
            gpio_write_pin(this->pin_ms3_, GPIO_PIN_RESET);
        }
    }

    void A4988::set_half_microstep() const noexcept
    {
        if (this->initialized_) {
            gpio_write_pin(this->pin_ms1_, GPIO_PIN_SET);
            gpio_write_pin(this->pin_ms2_, GPIO_PIN_RESET);
            gpio_write_pin(this->pin_ms3_, GPIO_PIN_RESET);
        }
    }

    void A4988::set_quarter_microstep() const noexcept
    {
        if (this->initialized_) {
            gpio_write_pin(this->pin_ms1_, GPIO_PIN_RESET);
            gpio_write_pin(this->pin_ms2_, GPIO_PIN_SET);
            gpio_write_pin(this->pin_ms3_, GPIO_PIN_RESET);
        }
    }

    void A4988::set_eighth_microstep() const noexcept
    {
        if (this->initialized_) {
            gpio_write_pin(this->pin_ms1_, GPIO_PIN_SET);
            gpio_write_pin(this->pin_ms2_, GPIO_PIN_SET);
            gpio_write_pin(this->pin_ms3_, GPIO_PIN_RESET);
        }
    }

    void A4988::set_sixteenth_microstep() const noexcept
    {
        if (this->initialized_) {
            gpio_write_pin(this->pin_ms1_, GPIO_PIN_RESET);
            gpio_write_pin(this->pin_ms2_, GPIO_PIN_RESET);
            gpio_write_pin(this->pin_ms3_, GPIO_PIN_SET);
        }
    }

    void A4988::set_direction(Direction const direction) const noexcept
    {
        switch (direction) {
            case Direction::FORWARD:
                this->set_forward_direction();
                break;
            case Direction::BACKWARD:
                this->set_backward_direction();
                break;
            default:
                break;
        }
    }

    void A4988::set_forward_direction() const noexcept
    {
        if (this->initialized_) {
            gpio_write_pin(this->pin_dir_, GPIO_PIN_RESET);
        }
    }

    void A4988::set_backward_direction() const noexcept
    {
        if (this->initialized_) {
            gpio_write_pin(this->pin_dir_, GPIO_PIN_SET);
        }
    }

    void A4988::set_reset(bool const reset) const noexcept
    {
        gpio_write_pin(this->pin_reset_, reset ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    void A4988::set_enable(bool const enable) const noexcept
    {
        gpio_write_pin(this->pin_enable_, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    void A4988::set_sleep(bool const sleep) const noexcept
    {
        gpio_write_pin(this->pin_sleep_, sleep ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    void A4988::set_step(bool const step) const noexcept
    {
        if (this->initialized_) {
            this->pwm_device_.set_compare_raw(step ? 10U : 0U);
        }
    }

    void A4988::initialize() noexcept
    {
        this->set_reset(false);
        this->set_enable(true);
        this->set_sleep(false);
        this->set_step(false);
        this->initialized_ = true;
    }

    void A4988::deinitialize() noexcept
    {
        this->set_reset(true);
        this->set_enable(false);
        this->set_sleep(true);
        this->set_step(false);
        this->initialized_ = false;
    }

}; // namespace A4988