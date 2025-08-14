#include "main.h"
#include "FreeRTOS.h"
#include "control_manager.hpp"
#include "event_bits.hpp"
#include "event_group_manager.hpp"
#include "gpio.h"
#include "i2c.h"
#include "imu_manager.hpp"
#include "log.hpp"
#include "log_manager.hpp"
#include "task.h"
#include "task_manager.hpp"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "wheel_manager.hpp"

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
    MX_USB_DEVICE_Init();

    segway::wheel_manager_init();
    segway::log_manager_init();
    segway::control_manager_init();
    segway::imu_manager_init();

    vTaskStartScheduler();
}
