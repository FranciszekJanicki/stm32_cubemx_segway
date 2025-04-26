#include "main_task.hpp"
#include "FreeRTOS.h"
#include "control_task.hpp"
#include "imu_task.hpp"
#include "log.hpp"
#include "task.h"
#include "wheel_task.hpp"
#include <array>

namespace {

    constexpr auto TAG = "main_task";

    constexpr auto TASK_PRIORITY = 1UL;
    constexpr auto TASK_STACK_DEPTH = 1024UL;
    constexpr auto TASK_NAME = "main_task";

    inline auto static_task = StaticTask_t{};
    inline auto task_stack = std::array<StackType_t, TASK_STACK_DEPTH>{};

    void task(void*) noexcept
    {
        segway::LOG(TAG, "main_task start");

        segway::control_task::task_init();
        segway::imu_task::task_init();
        segway::wheel_task::task_init();

        segway::LOG(TAG, "main_task end");

        vTaskDelete(nullptr);
    }

}; // namespace

namespace segway::main_task {

    void task_init() noexcept
    {
        xTaskCreateStatic(&task, TASK_NAME, task_stack.size(), nullptr, TASK_PRIORITY, task_stack.data(), &static_task);
    }

}; // namespace segway::main_task