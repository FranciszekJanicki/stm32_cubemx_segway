#ifndef A4988_CONFIG_HPP
#define A4988_CONFIG_HPP

#include "gpio.hpp"
#include "pwm_device.hpp"

namespace A4988 {

    using namespace STM32_Utility;

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
                break;
        }
    }

} // namespace A4988

#endif // A4988_CONFIG_HPP