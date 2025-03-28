#include "gpio.hpp"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM20948_INT_Pin) {
        gpio_pin6_exti = true;
    }
}

namespace Hardware {

    void initialize_gpio() noexcept
    {
        MX_GPIO_Init();
    }

    void deinitialize_gpio() noexcept
    {}

}; // namespace Hardware