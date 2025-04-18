#include "nvic.hpp"
#include "gpio.h"
#include "tim.h"
#include <cstdio>

using namespace Segway;

#ifdef __cplusplus
extern "C" {
#endif

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
    {
        if (htim->Instance == TIM2) {
            tim2_period_elapsed = true;
            std::puts("TIM2 PERIOD ELAPSED CALLBACK");
        }
    }

    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
    {
        if (htim->Instance == TIM1) {
            tim1_pulse_finished = true;
            std::puts("TIM1 PULSE FINISHED CALLBACK");
        }

        if (htim->Instance == TIM3) {
            std::puts("TIM3 PULSE FINISHED CALLBACK");
            tim3_pulse_finished = true;
        }
    }

    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        if (GPIO_Pin == ICM20948_INT_Pin) {
            std::puts("GPIO EXTI 6 CALLBACK\n\r");
            gpio_pin6_exti = true;
        }
    }

#ifdef __cplusplus
}
#endif
