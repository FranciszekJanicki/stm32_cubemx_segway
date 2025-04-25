#ifndef TIMERS_HPP
#define TIMERS_HPP

namespace segway {

    void sampling_timer_start() noexcept;
    void sampling_timer_stop() noexcept;

    void debounce_timer_start() noexcept;
    void debounce_timer_stop() noexcept;

}; // namespace segway

#endif // TIMERS_HPP