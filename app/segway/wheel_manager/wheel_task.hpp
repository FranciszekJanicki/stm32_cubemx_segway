#ifndef SEGWAY_WHEEL_TASK_HPP
#define SEGWAY_WHEEL_TASK_HPP

namespace segway {

    constexpr auto WHEEL_TASK_PRIORITY = 1UL;
    constexpr auto WHEEL_TASK_STACK_DEPTH = 1024UL;
    constexpr auto WHEEL_TASK_NAME = "wheel_task";
    constexpr auto WHEEL_TASK_ARG = nullptr;

    void wheel_task_init() noexcept;

} // namespace segway

#endif // SEGWAY_WHEEL_TASK_HPP