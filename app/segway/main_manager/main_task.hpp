#ifndef SEGWAY_MAIN_TASK_HPP
#define SEGWAY_MAIN_TASK_HPP

namespace segway {

    constexpr auto MAIN_TASK_PRIORITY = 1UL;
    constexpr auto MAIN_TASK_STACK_DEPTH = 1024UL;
    constexpr auto MAIN_TASK_NAME = "main_task";
    constexpr auto MAIN_TASK_ARG = nullptr;

    void main_task_init() noexcept;

} // namespace segway

#endif // SEGWAY_MAIN_TASK_HPP