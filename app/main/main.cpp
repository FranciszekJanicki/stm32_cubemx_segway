#include "main.h"
#include "gpio.hpp"
#include "hardware.hpp"
#include "pwm_device.hpp"
#include "segway.hpp"
#include "unit_test.hpp"

int main()
{
    HAL_Init();
    SystemClock_Config();

    Hardware::initialize_gpio();
    Hardware::initialize_uart2();
    Hardware::initialize_tim1();
    Hardware::initialize_tim2();
    Hardware::initialize_tim3();
    Hardware::initialize_i2c1();

    Segway::test_step_driver_1();
}
