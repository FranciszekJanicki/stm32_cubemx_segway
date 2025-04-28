#ifndef SEGWAY_IMU_TASK_HPP
#define SEGWAY_IMU_TASK_HPP

namespace segway {

    constexpr auto IMU_TASK_PRIORITY = 1UL;
    constexpr auto IMU_TASK_STACK_DEPTH = 1024UL;
    constexpr auto IMU_TASK_NAME = "control_task";
    constexpr auto IMU_TASK_ARG = nullptr;

    void imu_task_init() noexcept;

} // namespace segway

#endif // SEGWAY_IMU_TASK_HPP