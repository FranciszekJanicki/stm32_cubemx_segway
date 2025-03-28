#ifndef HARDWARE_GPIO_HPP
#define HARDWARE_GPIO_HPP

#include "gpio.h"

auto inline volatile gpio_pin6_exti = false;

namespace Hardware {

    void initialize_gpio() noexcept;

    void deinitialize_gpio() noexcept;

}; // namespace Hardware

#endif // HARDWARE_GPIO_HPP