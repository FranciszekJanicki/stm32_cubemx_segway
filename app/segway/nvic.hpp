#ifndef NVIC_HPP
#define NVIC_HPP

namespace Segway {

    auto inline volatile gpio_pin6_exti = false;
    auto inline volatile tim2_period_elapsed = false;
    auto inline volatile tim1_pulse_finished = false;
    auto inline volatile tim3_pulse_finished = false;

}; // namespace Segway

#endif // NVIC_HPP