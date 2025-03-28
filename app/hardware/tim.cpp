#include "tim.hpp"

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

namespace Hardware {

    void initialize_tim1() noexcept
    {
        MX_TIM1_Init();
    }

    void initialize_tim2() noexcept
    {
        MX_TIM2_Init();
    }

    void initialize_tim3() noexcept
    {
        MX_TIM3_Init();
    }

    void deinitialize_tim1() noexcept
    {}

    void deinitialize_tim2() noexcept
    {}

    void deinitialize_tim3() noexcept
    {}

}; // namespace Hardware
