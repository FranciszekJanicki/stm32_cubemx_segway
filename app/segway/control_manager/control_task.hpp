#ifndef SEGWAY_CONTROL_TASK_HPP
#define SEGWAY_CONTROL_TASK_HPP

namespace segway {

    constexpr auto CONTROL_TASK_PRIORITY = 1UL;
    constexpr auto CONTROL_TASK_STACK_DEPTH = 1024UL;
    constexpr auto CONTROL_TASK_NAME = "control_task";
    constexpr auto CONTROL_TASK_ARG = nullptr;

    void control_task_init() noexcept;

} // namespace segway

#endif // SEGWAY_CONTROL_TASK_HPP