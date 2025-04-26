#ifndef TIMERS_HPP
#define TIMERS_HPP

#include <cstdint>

namespace segway {

    void sampling_timer_start() noexcept;
    void sampling_timer_stop() noexcept;

    void motor1_pwm_timer_start() noexcept;
    void motor1_pwm_timer_stop() noexcept;

    void motor2_pwm_timer_start() noexcept;
    void motor2_pwm_timer_stop() noexcept;

    void motor1_step_timer_start() noexcept;
    void motor1_step_timer_stop() noexcept;
    void motor1_step_timer_set_frequency(std::uint32_t const frequency) noexcept;

    void motor2_step_timer_start() noexcept;
    void motor2_step_timer_stop() noexcept;
    void motor2_step_timer_set_frequency(std::uint32_t const frequency) noexcept;

}; // namespace segway

#endif // TIMERS_HPP