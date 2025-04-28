#include "control_task.hpp"
#include "FreeRTOS.h"
#include "control_manager.hpp"
#include "log.hpp"
#include "task.h"
#include "task_manager.hpp"
#include <array>

namespace segway {

    namespace {

        constexpr auto TAG = "control_task";

        inline auto control_static_task = StaticTask_t{};
        inline auto control_task_stack = std::array<StackType_t, CONTROL_TASK_STACK_DEPTH>{};

        void control_task(void*) noexcept
        {
            LOG(TAG, "control_task start");

            while (1) {
                control_manager_process();
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            LOG(TAG, "control_task end");
        }

    }; // namespace

    void control_task_init() noexcept
    {
        auto task_handle = xTaskCreateStatic(&control_task,
                                             CONTROL_TASK_NAME,
                                             control_task_stack.size(),
                                             CONTROL_TASK_ARG,
                                             CONTROL_TASK_PRIORITY,
                                             control_task_stack.data(),
                                             &control_static_task);

        set_task_handle(TaskType::CONTROL_TASK, task_handle);
    }

}; // namespace segway