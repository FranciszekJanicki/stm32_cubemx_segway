#ifndef A4988_CONFIG_HPP
#define A4988_CONFIG_HPP

#include <cstdint>
#include <stdfloat>

namespace a4988 {

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
        std::uint16_t pin_ms1;
        std::uint16_t pin_ms2;
        std::uint16_t pin_ms3;
        std::uint16_t pin_reset;
        std::uint16_t pin_sleep;
        std::uint16_t pin_dir;
        std::uint16_t pin_enable;
    };

    struct Interface {
        void* gpio_user;
        void (*gpio_init)(void*);
        void (*gpio_deinit)(void*);
        void (*gpio_write_pin)(void*, std::uint16_t, bool);

        void* pulse_user;
        void (*pulse_init)(void*);
        void (*pulse_deinit)(void*);
        void (*pulse_start)(void*);
        void (*pulse_stop)(void*);
        void (*pulse_set_freq)(void*, std::uint32_t);
    };

}; // namespace a4988

#endif // A4988_CONFIG_HPP