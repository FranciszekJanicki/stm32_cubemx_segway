#ifndef SEGWAY_TASK_MANAGER_HPP
#define SEGWAY_TASK_MANAGER_HPP

#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>

namespace segway {

    enum struct TaskType : std::uint8_t {
        CONTROL_TASK,
        IMU_TASK,
        WHEEL_TASK,
        TASK_NUM,
    };

    void set_task_handle(TaskType const type, TaskHandle_t const handle) noexcept;

    TaskHandle_t get_task_handle(TaskType const type) noexcept;

}; // namespace segway

#endif // SEGWAY_TASK_MANAGER_HPP