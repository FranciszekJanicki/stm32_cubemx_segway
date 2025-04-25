#include "timers.hpp"
#include "tim.h"

namespace segway {

    void sampling_timer_start() noexcept
    {
        HAL_TIM_Base_Start_IT(&htim2);
    }

    void sampling_timer_stop() noexcept
    {
        HAL_TIM_Base_Stop_IT(&htim2);
    }

    void debounce_timer_start() noexcept
    {
        HAL_TIM_Base_Start_IT(&htim4);
    }

    void debounce_timer_stop() noexcept
    {
        HAL_TIM_Base_Stop_IT(&htim4);
    }

}; // namespace segway