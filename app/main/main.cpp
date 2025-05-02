#include "main.h"
#include "FreeRTOS.h"
#include "control_manager.hpp"
#include "gpio.h"
#include "i2c.h"
#include "imu_manager.hpp"
#include "log.hpp"
#include "log_manager.hpp"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "wheel_manager.hpp"

#include "event_group_manager.hpp"

constexpr auto TAG = "NVIC";

#ifdef __cplusplus
extern "C" {
#endif

// void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
// {
//     std::puts("HAL_I2C_MemTxCpltCallback");

//     auto task_woken = pdFALSE;

//     if (hi2c->Instance == I2C1) {
//         xEventGroupSetBitsFromISR(segway::get_imu_event_group(),
//                                   segway::IMUEventBit::TX_COMPLETE,
//                                   &task_woken);
//     }

//     portYIELD_FROM_ISR(task_woken);
// }

// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
// {
//     std::puts("HAL_I2C_MemRxCpltCallback");

//     auto task_woken = pdFALSE;

//     if (hi2c->Instance == I2C1) {
//         xEventGroupSetBitsFromISR(segway::get_imu_event_group(),
//                                   segway::IMUEventBit::RX_COMPLETE,
//                                   &task_woken);
//     }

//     portYIELD_FROM_ISR(task_woken);
// }

// void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
// {
//     std::puts("HAL_TIM_PWM_PulseFinishedCallback");

//     auto task_woken = pdFALSE;

//     if (htim->Instance == TIM1) {
//         xEventGroupSetBitsFromISR(segway::get_wheel_event_group(),
//                                   segway::WheelEventBit::LEFT_PWM_PULSE,
//                                   &task_woken);
//     } else if (htim->Instance == TIM3) {
//         xEventGroupSetBitsFromISR(segway::get_wheel_event_group(),
//                                   segway::WheelEventBit::RIGHT_PWM_PULSE,
//                                   &task_woken);
//     }

//     portYIELD_FROM_ISR(task_woken);
// }

// void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
// {
//     std::puts("HAL_I2C_ErrorCallback");

//     auto task_woken = pdFALSE;

//     if (hi2c->Instance == I2C1) {
//         xEventGroupSetBitsFromISR(segway::get_imu_event_group(),
//                                   segway::IMUEventBit::I2C_ERROR,
//                                   &task_woken);
//     }

//     portYIELD_FROM_ISR(task_woken);
// }

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
// {
//     std::puts("HAL_TIM_PeriodElapsedCallback");

//     auto task_woken = pdFALSE;

//     if (htim->Instance == TIM1) {
//         xEventGroupSetBitsFromISR(segway::get_wheel_event_group(),
//                                   segway::WheelEventBit::LEFT_STEP_TIMER,
//                                   &task_woken);
//     } else if (htim->Instance == TIM2) {
//         xEventGroupSetBitsFromISR(segway::get_imu_event_group(),
//                                   segway::IMUEventBit::SAMPLING_TIMER,
//                                   &task_woken);
//     } else if (htim->Instance == TIM3) {
//         xEventGroupSetBitsFromISR(segway::get_wheel_event_group(),
//                                   segway::WheelEventBit::RIGHT_STEP_TIMER,
//                                   &task_woken);
//     } else if (htim->Instance == TIM4) {
//         HAL_IncTick();
//     }

//     portYIELD_FROM_ISR(task_woken);
// }

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     std::puts("HAL_GPIO_EXTI_Callback");

//     auto task_woken = pdFALSE;

//     if (GPIO_Pin == (1 << 6)) {
//         xEventGroupSetBitsFromISR(segway::get_imu_event_group(),
//                                   segway::IMUEventBit::DATA_READY,
//                                   &task_woken);
//     }

//     portYIELD_FROM_ISR(task_woken);
// }

#ifdef __cplusplus
};
#endif

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

    segway::log_manager_init();
    segway::control_manager_init();
    segway::imu_manager_init();
    segway::wheel_manager_init();

    vTaskStartScheduler();
}
