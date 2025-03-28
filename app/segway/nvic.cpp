#include "nvic.hpp"
#include "../../Core/Inc/gpio.h"
#include "../../Core/Inc/tim.h"

using namespace Segway;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2) {
        tim2_period_elapsed = true;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        tim1_pulse_finished = true;
    }

    if (htim->Instance == TIM3) {
        tim3_pulse_finished = true;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM20948_INT_Pin) {
        gpio_pin6_exti = true;
    }
}
