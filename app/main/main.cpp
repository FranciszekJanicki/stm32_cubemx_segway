#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "unit_test.hpp"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

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
    MX_USB_DEVICE_Init();

    HAL_Delay(2000);

    Segway::test_segway();
}
