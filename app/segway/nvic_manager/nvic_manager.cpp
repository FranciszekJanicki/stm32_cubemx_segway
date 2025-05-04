#include "FreeRTOS.h"
#include "event_group_manager.hpp"
#include "event_groups.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include <utility>

using namespace segway;

#ifdef __cplusplus
extern "C" {
#endif

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::TX_COMPLETE,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::RX_COMPLETE,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::I2C_ERROR,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    auto task_woken = pdFALSE;

    if (htim->Instance == TIM1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::WHEEL),
                                  WheelEventBit::LEFT_STEP_TIMER,
                                  &task_woken);
    } else if (htim->Instance == TIM2) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::DATA_READY,
                                  &task_woken);
    } else if (htim->Instance == TIM3) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::WHEEL),
                                  WheelEventBit::RIGHT_STEP_TIMER,
                                  &task_woken);
    } else if (htim->Instance == TIM4) {
        HAL_IncTick();
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    auto task_woken = pdFALSE;

    if (GPIO_Pin == (1 << 6)) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::DATA_READY,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

#ifdef __cplusplus
};
#endif
