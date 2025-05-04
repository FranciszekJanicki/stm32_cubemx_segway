#include "main.h"
#include "FreeRTOS.h"
#include "control_manager.hpp"
#include "event_group_manager.hpp"
#include "gpio.h"
#include "i2c.h"
#include "imu_manager.hpp"
#include "log.hpp"
#include "log_manager.hpp"
#include "main_manager.hpp"
#include "task.h"
#include "task_manager.hpp"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "wheel_manager.hpp"

using namespace segway;

#ifdef __cplusplus
extern "C" {
#endif

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xTaskNotifyFromISR(get_task(TaskType::IMU),
                           IMUEventBit::TX_COMPLETE,
                           eNotifyAction::eSetBits,
                           &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xTaskNotifyFromISR(get_task(TaskType::IMU),
                           IMUEventBit::RX_COMPLETE,
                           eNotifyAction::eSetBits,
                           &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xTaskNotifyFromISR(get_task(TaskType::IMU),
                           IMUEventBit::I2C_ERROR,
                           eNotifyAction::eSetBits,
                           &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    auto task_woken = pdFALSE;

    if (htim->Instance == TIM1) {
        xTaskNotifyFromISR(get_task(TaskType::WHEEL),
                           WheelEventBit::LEFT_STEP_TIMER,
                           eNotifyAction::eSetBits,
                           &task_woken);
    } else if (htim->Instance == TIM2) {
        xTaskNotifyFromISR(get_task(TaskType::IMU),
                           IMUEventBit::DATA_READY,
                           eNotifyAction::eSetBits,
                           &task_woken);
    } else if (htim->Instance == TIM3) {
        xTaskNotifyFromISR(get_task(TaskType::WHEEL),
                           WheelEventBit::RIGHT_STEP_TIMER,
                           eNotifyAction::eSetBits,
                           &task_woken);
    } else if (htim->Instance == TIM4) {
        HAL_IncTick();
        task_woken = true;
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    auto task_woken = pdFALSE;

    if (GPIO_Pin == (1 << 6)) {
        xTaskNotifyFromISR(get_task(TaskType::IMU),
                           IMUEventBit::DATA_READY,
                           eNotifyAction::eSetBits,
                           &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

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
    //  MX_USB_DEVICE_Init();

    // segway::main_manager_init();
    segway::log_manager_init();
    // segway::control_manager_init();
    // segway::wheel_manager_init();
    // segway::imu_manager_init();

    vTaskStartScheduler();
}
