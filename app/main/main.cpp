#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "unit_test.hpp"
#include "usart.h"

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    Segway::test_step_driver_1();
}
