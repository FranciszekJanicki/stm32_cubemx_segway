#include "imu_task.hpp"
#include "FreeRTOS.h"
#include "log.hpp"
#include "task.h"
#include <array>

namespace {

    constexpr auto TAG = "imu_task";

    constexpr auto TASK_PRIORITY = 1UL;
    constexpr auto TASK_STACK_DEPTH = 1024UL;
    constexpr auto TASK_NAME = "imu_task";

    inline auto static_task = StaticTask_t{};
    inline auto task_stack = std::array<StackType_t, TASK_STACK_DEPTH>{};

    void task(void*) noexcept
    {
        segway::LOG(TAG, "imu_task start");

        while (1) {
            segway::LOG(TAG, "IMU task here!");
        }

        segway::LOG(TAG, "imu_task end");
    }

}; // namespace

namespace segway::imu_task {

    void task_init() noexcept
    {
        xTaskCreateStatic(&task, TASK_NAME, task_stack.size(), nullptr, TASK_PRIORITY, task_stack.data(), &static_task);
    }

}; // namespace segway::imu_task