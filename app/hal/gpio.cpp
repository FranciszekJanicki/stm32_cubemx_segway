#include "gpio.hpp"
#include <array>
#include <utility>

namespace hal {

    inline GPIO_TypeDef* gpio_pin_to_port(GPIOPin const pin) noexcept
    {
        // constexpr auto GPIO_ADDRESSES =
        //     std::array{GPIOAddr::PORT_A, GPIOAddr::PORT_B, GPIOAddr::PORT_C};

        // auto index = std::to_underlying(pin) / 16U;
        // return (index < GPIO_ADDRESSES.size()) ?
        // reinterpret_cast<GPIOPort*>(GPIO_ADDRESSES[index])
        //                                        : nullptr;

        constexpr auto GPIO_ADDRESSES = std::array{GPIOA, GPIOB, GPIOC};

        auto index = std::to_underlying(pin) / 16U;
        return (index < GPIO_ADDRESSES.size()) ? GPIO_ADDRESSES[index] : nullptr;
    }

    inline std::uint16_t gpio_pin_to_mask(GPIOPin const pin) noexcept
    {
        return 1U << (std::to_underlying(pin) % 16U);
    }

    GPIOState gpio_read_pin(GPIOPin const pin) noexcept
    {
        if (pin != GPIOPin::NC) {
            auto port = gpio_pin_to_port(pin);
            auto mask = gpio_pin_to_mask(pin);

            // return (port->IDR & mask) ? GPIOState::SET :
            // GPIOState::RESET);
            return static_cast<GPIOState>(HAL_GPIO_ReadPin(port, mask));
        } else {
            return GPIOState::UNDEFINED;
        }
    }

    void gpio_set_pin(GPIOPin const pin) noexcept
    {
        if (pin != GPIOPin::NC) {
            auto port = gpio_pin_to_port(pin);
            auto mask = gpio_pin_to_mask(pin);

            //  port->BSRR = mask;
            HAL_GPIO_WritePin(port, mask, GPIO_PIN_SET);
        }
    }

    void gpio_reset_pin(GPIOPin const pin) noexcept
    {
        if (pin != GPIOPin::NC) {
            auto port = gpio_pin_to_port(pin);
            auto mask = gpio_pin_to_mask(pin);

            //  port->BSRR = (mask << 16);
            HAL_GPIO_WritePin(port, mask, GPIO_PIN_RESET);
        }
    }

    void gpio_write_pin(GPIOPin const pin, GPIOState const state) noexcept
    {
        if (pin != GPIOPin::NC) {
            auto port = gpio_pin_to_port(pin);
            auto mask = gpio_pin_to_mask(pin);

            if (GPIOState::SET == state) {
                gpio_set_pin(pin);
            } else if (GPIOState::RESET == state) {
                gpio_reset_pin(pin);
            }
        }
    }

    void gpio_toggle_pin(GPIOPin const pin) noexcept
    {
        if (pin != GPIOPin::NC) {
            auto port = gpio_pin_to_port(pin);
            auto mask = gpio_pin_to_mask(pin);

            // port->ODR ^= mask;
            HAL_GPIO_TogglePin(port, mask);
        }
    }

}; // namespace hal