#ifndef A4988_CONFIG_HPP
#define A4988_CONFIG_HPP

#include "gpio.hpp"
#include "pwm_device.hpp"
#include <stdfloat>

namespace a4988 {

    using namespace stm32_utility;

    enum struct Microstep : std::uint8_t {
        FULL,
        HALF,
        QUARTER,
        EIGHTH,
        SIXTEENTH,
    };

    enum struct Direction : std::uint8_t {
        FORWARD,
        BACKWARD,
        STOP,
    };

    inline std::float64_t microstep_to_fraction(Microstep const microstep) noexcept
    {
        switch (microstep) {
            case Microstep::FULL:
                return 1.0F;
            case Microstep::HALF:
                return 0.5F;
            case Microstep::QUARTER:
                return 0.25F;
            case Microstep::EIGHTH:
                return 0.125F;
            case Microstep::SIXTEENTH:
                return 0.0625F;
            default:
                return 0.0F;
        }
    }

    struct Config {
        PWMDevice pwm_device = {};

        GPIO pin_ms1 = GPIO::NC;
        GPIO pin_ms2 = GPIO::NC;
        GPIO pin_ms3 = GPIO::NC;
        GPIO pin_reset = GPIO::NC;
        GPIO pin_sleep = GPIO::NC;
        GPIO pin_dir = GPIO::NC;
        GPIO pin_enable = GPIO::NC;
    };

}; // namespace a4988

#endif // A4988_CONFIG_HPP