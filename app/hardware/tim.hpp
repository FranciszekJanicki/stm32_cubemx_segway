#ifndef TIM_HPP
#define TIM_HPP

#include "tim.h"

auto inline volatile tim2_period_elapsed = false;
auto inline volatile tim1_pulse_finished = false;
auto inline volatile tim3_pulse_finished = false;

namespace Hardware {

    void initialize_tim1() noexcept;

    void initialize_tim2() noexcept;

    void initialize_tim3() noexcept;

    void deinitialize_tim1() noexcept;

    void deinitialize_tim2() noexcept;

    void deinitialize_tim3() noexcept;

}; // namespace Hardware

#endif // TIM_HPP