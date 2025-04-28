#include "timers.hpp"
#include "tim.h"
#include "utility.hpp"

namespace segway {

    void sampling_timer_start() noexcept
    {
        HAL_TIM_Base_Start_IT(&htim2);
    }

    void sampling_timer_stop() noexcept
    {
        HAL_TIM_Base_Stop_IT(&htim2);
    }

    void motor1_pwm_timer_start() noexcept
    {
        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    }

    void motor1_pwm_timer_stop() noexcept
    {
        HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
    }

    void motor2_pwm_timer_start() noexcept
    {
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
    }

    void motor2_pwm_timer_stop() noexcept
    {
        HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    }

    void motor1_step_timer_start() noexcept
    {
        HAL_TIM_Base_Start_IT(&htim1);
    }

    void motor1_step_timer_stop() noexcept
    {
        HAL_TIM_Base_Stop_IT(&htim1);
    }

    void motor1_step_timer_set_frequency(std::uint32_t const frequency) noexcept
    {
        if (frequency > 0UL) {
            auto prescaler = 0UL;
            auto period = 0UL;

            utility::frequency_to_prescaler_and_period(frequency, 84000000, 0, 0xFFFF, 0xFFFF, prescaler, period);

            htim1.Instance->PSC = prescaler;
            htim1.Instance->ARR = period;
        }
    }

    void motor2_step_timer_start() noexcept
    {
        HAL_TIM_Base_Start_IT(&htim3);
    }

    void motor2_step_timer_stop() noexcept
    {
        HAL_TIM_Base_Stop_IT(&htim3);
    }

    void motor2_step_timer_set_frequency(std::uint32_t const frequency) noexcept
    {
        if (frequency > 0UL) {
            auto prescaler = 0UL;
            auto period = 0UL;

            utility::frequency_to_prescaler_and_period(frequency, 84000000, 0, 0xFFFF, 0xFFFF, prescaler, period);

            htim3.Instance->PSC = prescaler;
            htim3.Instance->ARR = period;
        }
    }

}; // namespace segway