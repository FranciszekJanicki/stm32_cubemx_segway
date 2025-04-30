#include "FreeRTOS.h"
#include "event_group_manager.hpp"
#include "event_groups.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include <utility>

using namespace segway;

extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::TX_COMPLETE,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::RX_COMPLETE,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    auto task_woken = pdFALSE;

    if (htim->Instance == TIM1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::WHEEL),
                                  WheelEventBit::LEFT_PWM_PULSE,
                                  &task_woken);
    } else if (htim->Instance == TIM3) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::WHEEL),
                                  WheelEventBit::RIGHT_PWM_PULSE,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::I2C_ERROR,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

#include "log.hpp"

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    auto task_woken = pdFALSE;

    if (htim->Instance == TIM1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::WHEEL),
                                  WheelEventBit::LEFT_STEP_TIMER,
                                  &task_woken);
    } else if (htim->Instance == TIM2) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::SAMPLING_TIMER,
                                  &task_woken);
    } else if (htim->Instance == TIM3) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::WHEEL),
                                  WheelEventBit::RIGHT_STEP_TIMER,
                                  &task_woken);
    } else if (htim->Instance == TIM4) {
        HAL_IncTick();
    }

    LOG("NVIC", "DUPA");

    portYIELD_FROM_ISR(task_woken);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    auto task_woken = pdFALSE;

    if (GPIO_Pin == ICM20948_INT_Pin) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::DATA_READY,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}
