#include "main_task.hpp"
#include "FreeRTOS.h"
#include "control_queue.hpp"
#include "control_task.hpp"
#include "imu_event_group.hpp"
#include "imu_task.hpp"
#include "log.hpp"
#include "task.h"
#include "wheel_event_group.hpp"
#include "wheel_queue.hpp"
#include "wheel_task.hpp"
#include <array>

namespace segway {

    namespace {

        constexpr auto TAG = "main_task";

        inline auto default_static_task = StaticTask_t{};
        inline auto main_task_stack = std::array<StackType_t, MAIN_TASK_STACK_DEPTH>{};

        void main_task(void*) noexcept
        {
            LOG(TAG, "main_task start");

            control_queue_init();
            imu_event_group_init();
            wheel_event_group_init();
            wheel_queue_init();

            control_task_init();
            imu_task_init();
            wheel_task_init();

            LOG(TAG, "main_task end");

            vTaskDelete(nullptr);
        }

    }; // namespace

    void main_task_init() noexcept
    {
        xTaskCreateStatic(&main_task,
                          MAIN_TASK_NAME,
                          main_task_stack.size(),
                          MAIN_TASK_ARG,
                          MAIN_TASK_PRIORITY,
                          main_task_stack.data(),
                          &default_static_task);
    }

}; // namespace segway