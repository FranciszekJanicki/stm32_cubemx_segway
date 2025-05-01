#include "FreeRTOS.h"
#include "event_group_manager.hpp"
#include "event_groups.h"
#include "gpio.h"
#include "i2c.h"
#include "log.hpp"
#include "tim.h"
#include <utility>

using namespace segway;

namespace {

    constexpr auto TAG = "NVIC";

};

extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    LOG(TAG, "%s", "HAL_I2C_MemTxCpltCallback");

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
    LOG(TAG, "%s", "HAL_I2C_MemRxCpltCallback");

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
    LOG(TAG, "%s", "HAL_TIM_PWM_PulseFinishedCallback");

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
    LOG(TAG, "%s", "HAL_I2C_ErrorCallback");

    auto task_woken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::I2C_ERROR,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    LOG(TAG, "%s", "HAL_TIM_PeriodElapsedCallback");

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

    portYIELD_FROM_ISR(task_woken);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    LOG(TAG, "%s", "HAL_GPIO_EXTI_Callback");

    auto task_woken = pdFALSE;

    if (GPIO_Pin == (1 << 6)) {
        xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU),
                                  IMUEventBit::DATA_READY,
                                  &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}
