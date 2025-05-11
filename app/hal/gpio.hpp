#ifndef GPIO_HPP
#define GPIO_HPP

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include <cstdint>

namespace hal {

    enum struct GPIOPin : std::uint16_t {
        NC = static_cast<std::uint8_t>(-1),
        PA0 = 0,
        PA1,
        PA2,
        PA3,
        PA4,
        PA5,
        PA6,
        PA7,
        PA8,
        PA9,
        PA10,
        PA11,
        PA12,
        PA13,
        PA14,
        PA15,
        PB0,
        PB1,
        PB2,
        PB3,
        PB4,
        PB5,
        PB6,
        PB7,
        PB8,
        PB9,
        PB10,
        PB11,
        PB12,
        PB13,
        PB14,
        PB15,
        PC0,
        PC1,
        PC2,
        PC3,
        PC4,
        PC5,
        PC6,
        PC7,
        PC8,
        PC9,
        PC10,
        PC11,
        PC12,
        PC13,
        PC14,
        PC15,
        PD0,
        PD1,
        PD2,
        PD3,
        PD4,
        PD5,
        PD6,
        PD7,
        PD8,
        PD9,
        PD10,
        PD11,
        PD12,
        PD13,
        PD14,
        PD15,
        //
        PE0,
        PE1,
        PE2,
        PE3,
        PE4,
        PE5,
        PE6,
        PE7,
        PE8,
        PE9,
        PE10,
        PE11,
        PE12,
        PE13,
        PE14,
        PE15,
        PF0,
        PF1,
        PF2,
        PF3,
        PF4,
        PF5,
        PF6,
        PF7,
        PF8,
        PF9,
        PF10,
        PF11,
        PF12,
        PF13,
        PF14,
        PF15,
        PG0,
        PG1,
        PG2,
        PG3,
        PG4,
        PG5,
        PG6,
        PG7,
        PG8,
        PG9,
        PG10,
        PG11,
        PG12,
        PG13,
        PG14,
        PG15,
        PH0,
        PH1,
        PH2,
        PH3,
        PH4,
        PH5,
        PH6,
        PH7,
        PH8,
        PH9,
        PH10,
        PH11,
        PH12,
        PH13,
        PH14,
        PH15,
    };

    enum struct GPIOState : std::uint8_t {
        UNDEFINED = static_cast<std::uint8_t>(-1),
        SET = 1,
        RESET = 0,
    };

    enum struct GPIOAddr : std::uintptr_t {
        PORT_A = GPIOA_BASE,
        PORT_B = GPIOA_BASE,
        PORT_C = GPIOA_BASE,
    };

    struct GPIOPort {
        std::uint32_t volatile MODER;
        std::uint32_t volatile OTYPER;
        std::uint32_t volatile OSPEEDR;
        std::uint32_t volatile PUPDR;
        std::uint32_t volatile IDR;
        std::uint32_t volatile ODR;
        std::uint32_t volatile BSRR;
        std::uint32_t volatile LCKR;
        std::uint32_t volatile AFR[2];
    };

    GPIOState gpio_read_pin(GPIOPin const pin) noexcept;

    void gpio_set_pin(GPIOPin const pin) noexcept;
    void gpio_reset_pin(GPIOPin const pin) noexcept;
    void gpio_write_pin(GPIOPin const pin, GPIOState const state) noexcept;
    void gpio_toggle_pin(GPIOPin const pin) noexcept;

}; // namespace hal

#endif // GPIO_HPP