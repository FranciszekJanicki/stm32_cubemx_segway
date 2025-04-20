#include "nvic.hpp"
#include "gpio.h"
#include "i2c.h"
#include "log.hpp"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

using namespace Segway;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        nvic_mask.motor1_pwm_pulse = true;
    } else if (htim->Instance == TIM3) {
        nvic_mask.motor2_pwm_pulse = true;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1) {
        nvic_mask.i2c_error = true;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2) {
        nvic_mask.sampling_timer = true;
    } else if (htim->Instance == TIM4) {
        HAL_TIM_Base_Stop_IT(&htim4);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
            nvic_mask.data_ready = true;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_6) {
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        HAL_TIM_Base_Start_IT(&htim4);
    }
}

#ifdef __cplusplus
}
#endif
