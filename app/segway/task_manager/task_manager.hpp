#ifndef SEGWAY_TASK_MANAGER_HPP
#define SEGWAY_TASK_MANAGER_HPP

#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>

namespace segway {

    enum struct TaskType : std::uint8_t {
        CONTROL,
        IMU,
        WHEEL,
        TASK_NUM,
    };

    void set_task(TaskType const type, TaskHandle_t const handle) noexcept;

    inline void set_control_task(TaskHandle_t const handle) noexcept
    {
        set_task(TaskType::CONTROL, handle);
    }

    inline void set_imu_task(TaskHandle_t const handle) noexcept
    {
        set_task(TaskType::IMU, handle);
    }

    inline void set_wheel_task(TaskHandle_t const handle) noexcept
    {
        set_task(TaskType::WHEEL, handle);
    }

    TaskHandle_t get_task(TaskType const type) noexcept;

    inline TaskHandle_t get_control_task() noexcept
    {
        return get_task(TaskType::CONTROL);
    }

    inline TaskHandle_t get_imu_task() noexcept
    {
        return get_task(TaskType::IMU);
    }

    inline TaskHandle_t get_wheel_task() noexcept
    {
        return get_task(TaskType::WHEEL);
    }

}; // namespace segway

#endif // SEGWAY_TASK_MANAGER_HPP