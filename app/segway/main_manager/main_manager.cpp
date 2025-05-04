#include "main_manager.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "log.hpp"
#include "queue_manager.hpp"
#include "task.h"
#include "tim.h"
#include <cassert>

namespace segway {

    namespace {

        constexpr auto TAG = "main_manager";

        void main_task(void*) noexcept
        {
            LOG(TAG, "main_task start");

            auto event = IMUEvent{.type = IMUEventType::START};
            xQueueSend(get_queue(QueueType::IMU), &event, pdMS_TO_TICKS(1));

            LOG(TAG, "main_task end");

            vTaskDelete(nullptr);
        }

        inline void main_task_init() noexcept
        {
            constexpr auto MAIN_TASK_PRIORITY = 1UL;
            constexpr auto MAIN_TASK_STACK_DEPTH = 1024UL;
            constexpr auto MAIN_TASK_NAME = "main_task";
            constexpr auto MAIN_TASK_ARG = nullptr;

            static auto main_static_task = StaticTask_t{};
            static auto main_task_stack = std::array<StackType_t, MAIN_TASK_STACK_DEPTH>{};

            xTaskCreateStatic(&main_task,
                              MAIN_TASK_NAME,
                              main_task_stack.size(),
                              MAIN_TASK_ARG,
                              MAIN_TASK_PRIORITY,
                              main_task_stack.data(),
                              &main_static_task);
        }

    }; // namespace

    void main_manager_init() noexcept
    {
        main_task_init();
    }

}; // namespace segway
