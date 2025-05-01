#include "main.h"
#include "FreeRTOS.h"
#include "gpio.h"
#include "i2c.h"
#include "main_task.hpp"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "wwdg.h"

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_WWDG_Init();
    MX_USB_DEVICE_Init();

    segway::main_task_init();
    vTaskStartScheduler();
}
