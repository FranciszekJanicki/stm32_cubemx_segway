#include "wheel_task.hpp"
#include "FreeRTOS.h"
#include "log.hpp"
#include "task.h"
#include "task_manager.hpp"
#include "wheel_manager.hpp"
#include <array>

namespace segway {

    namespace {

        constexpr auto TAG = "wheel_task";

        inline auto wheel_static_task = StaticTask_t{};
        inline auto wheel_task_stack = std::array<StackType_t, WHEEL_TASK_STACK_DEPTH>{};

        void wheel_task(void*) noexcept
        {
            LOG(TAG, "wheel_task start");

            while (1) {
                wheel_manager_process();
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            LOG(TAG, "wheel_task end");
        }

    }; // namespace

    void wheel_task_init() noexcept
    {
        auto task_handle = xTaskCreateStatic(&wheel_task,
                                             WHEEL_TASK_NAME,
                                             wheel_task_stack.size(),
                                             WHEEL_TASK_ARG,
                                             WHEEL_TASK_PRIORITY,
                                             wheel_task_stack.data(),
                                             &wheel_static_task);

        set_task_handle(TaskType::WHEEL_TASK, task_handle);
    }

}; // namespace segway