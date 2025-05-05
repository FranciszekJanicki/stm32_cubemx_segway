#include "FreeRTOS.h"
#include "event_bits.hpp"
#include "event_group_manager.hpp"
#include "gpio.h"
#include "i2c.h"
#include "task_manager.hpp"
#include "tim.h"
#include <utility>

using namespace segway;

#ifdef __cplusplus
extern "C" {
#endif

inline bool set_control_event_bits_from_isr(std::uint32_t const event_bits) noexcept
{
    auto task_woken = pdFALSE;
#ifdef USE_EVENT_GROUPS
    xEventGroupSetBitsFromISR(get_event_group(EventGroupType::CONTROL), event_bits, &task_woken);
#else
    xTaskNotifyFromISR(get_task(TaskType::CONTROL),
                       event_bits,
                       eNotifyAction::eSetBits,
                       &task_woken);
#endif
    return task_woken;
}

inline bool set_imu_event_bits_from_isr(std::uint32_t const event_bits) noexcept
{
    auto task_woken = pdFALSE;
#ifdef USE_EVENT_GROUPS
    xEventGroupSetBitsFromISR(get_event_group(EventGroupType::IMU), event_bits, &task_woken);
#else
    xTaskNotifyFromISR(get_task(TaskType::IMU), event_bits, eNotifyAction::eSetBits, &task_woken);
#endif
    return task_woken;
}

inline bool set_wheel_event_bits_from_isr(std::uint32_t const event_bits) noexcept
{
    auto task_woken = pdFALSE;
#ifdef USE_EVENT_GROUPS
    xEventGroupSetBitsFromISR(get_event_group(EventGroupType::WHEEL), event_bits, &task_woken);
#else
    xTaskNotifyFromISR(get_task(TaskType::WHEEL), event_bits, eNotifyAction::eSetBits, &task_woken);
#endif
    return task_woken;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1) {
        portYIELD_FROM_ISR(set_imu_event_bits_from_isr(IMUEventBit::TX_COMPLETE));
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1) {
        portYIELD_FROM_ISR(set_imu_event_bits_from_isr(IMUEventBit::RX_COMPLETE));
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1) {
        portYIELD_FROM_ISR(set_imu_event_bits_from_isr(IMUEventBit::I2C_ERROR));
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        portYIELD_FROM_ISR(set_wheel_event_bits_from_isr(WheelEventBit::LEFT_STEP_TIMER));
    } else if (htim->Instance == TIM2) {
        portYIELD_FROM_ISR(set_imu_event_bits_from_isr(IMUEventBit::DATA_READY));
    } else if (htim->Instance == TIM3) {
        portYIELD_FROM_ISR(set_wheel_event_bits_from_isr(WheelEventBit::RIGHT_STEP_TIMER));
    } else if (htim->Instance == TIM4) {
        HAL_IncTick();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == (1 << 6)) {
        portYIELD_FROM_ISR(set_imu_event_bits_from_isr(IMUEventBit::DATA_READY));
    }
}

#ifdef __cplusplus
};
#endif
