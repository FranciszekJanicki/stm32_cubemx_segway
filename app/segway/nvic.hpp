#ifndef NVIC_HPP
#define NVIC_HPP

#include <cstdint>

#include <cstdint>

namespace Segway {

    struct NVICMask {
        std::uint8_t : 3;
        std::uint8_t data_ready : 1;
        std::uint8_t motor1_pwm_pulse : 1;
        std::uint8_t motor2_pwm_pulse : 1;
        std::uint8_t i2c_error : 1;
        std::uint8_t sampling_timer : 1;
    } __attribute__((__packed__));

    inline auto volatile nvic_mask = NVICMask{};

}; // namespace Segway

#endif // NVIC_HPP