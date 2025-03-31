#ifndef NVIC_HPP
#define NVIC_HPP

namespace Segway {

    inline auto volatile gpio_pin6_exti = false;
    inline auto volatile tim2_period_elapsed = false;
    inline auto volatile tim1_pulse_finished = false;
    inline auto volatile tim3_pulse_finished = false;

}; // namespace Segway

#endif // NVIC_HPP