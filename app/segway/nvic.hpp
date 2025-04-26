#ifndef NVIC_HPP
#define NVIC_HPP

#include <cstdint>

#include <cstdint>

#define PACKED __attribute__((__packed__))

namespace segway {

    struct NVICMask {
        std::uint8_t data_ready : 1;
        std::uint8_t motor1_pwm_pulse : 1;
        std::uint8_t motor2_pwm_pulse : 1;
        std::uint8_t i2c_error : 1;
        std::uint8_t sampling_timer : 1;
        std::uint8_t motor1_step_timer : 1;
        std::uint8_t motor2_step_timer : 1;
        std::uint8_t i2c_rx_cplt : 1;
        std::uint8_t i2c_tx_cplt : 1;
    } PACKED;

    inline auto volatile nvic_mask = NVICMask{};

}; // namespace segway

#undef PACKED

#endif // NVIC_HPP