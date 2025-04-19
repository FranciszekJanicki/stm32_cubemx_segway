#include "nvic.hpp"
#include "gpio.h"
#include "log.hpp"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace {

    using namespace Segway;

    constexpr auto TAG = "NVIC";
    constexpr auto log_nvic = false;

}; // namespace

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        if constexpr (log_nvic) {
            LOG(TAG, "TIM1 PULSE FINISHED CALLBACK\n\r");
        }

        tim1_pulse_finished = true;
    }

    if (htim->Instance == TIM3) {
        if constexpr (log_nvic) {
            LOG(TAG, "TIM3 PULSE FINISHED CALLBACK\n\r");
        }

        tim3_pulse_finished = true;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1) {
        if constexpr (log_nvic) {
            LOG(TAG, "I2C ERROR CALLBACK\n\r");
        }

        i2c1_error = true;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2) {
        if constexpr (log_nvic) {
            LOG(TAG, "TIM2 PERIOD ELAPSED CALLBACK\n\r");
        }

        gpio_pin6_exti = true;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM20948_INT_Pin) {
        if constexpr (log_nvic) {
            LOG(TAG, "GPIO EXTI 6 CALLBACK\n\r");
        }

        gpio_pin6_exti = true;
    }
}

#ifdef __cplusplus
}
#endif
