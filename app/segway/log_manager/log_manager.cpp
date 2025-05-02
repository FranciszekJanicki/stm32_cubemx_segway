#include "log_manager.hpp"
#include "FreeRTOS.h"
#include "queue_manager.hpp"
#include "semphr.h"
#include "task.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <array>
#include <cstring>

namespace segway {

    namespace {

        void log_task(void*) noexcept
        {
            auto event = LogEvent{};

            while (1) {
                while (uxQueueMessagesWaiting(get_log_queue())) {
                    if (xQueueReceive(get_log_queue(), &event, pdMS_TO_TICKS(10))) {
                        auto msg = reinterpret_cast<std::uint8_t*>(event.buffer);
                        auto msg_len = std::strlen(event.buffer);

                        HAL_UART_Transmit(&huart2, msg, msg_len, 100);
                        // CDC_Transmit_FS(msg, msg_len);
                    }
                }
            }
        }

        inline void log_queue_init() noexcept
        {
            constexpr auto LOG_QUEUE_ITEM_SIZE = sizeof(LogEvent);
            constexpr auto LOG_QUEUE_ITEMS = 20UL;
            constexpr auto LOG_QUEUE_STORAGE_SIZE = LOG_QUEUE_ITEM_SIZE * LOG_QUEUE_ITEMS;

            static auto log_static_queue = StaticQueue_t{};
            static auto log_queue_storage = std::array<std::uint8_t, LOG_QUEUE_STORAGE_SIZE>{};

            set_log_queue(xQueueCreateStatic(LOG_QUEUE_ITEMS,
                                             LOG_QUEUE_ITEM_SIZE,
                                             log_queue_storage.data(),
                                             &log_static_queue));
        }

        inline void log_task_init() noexcept
        {
            constexpr auto LOG_TASK_PRIORITY = 1UL;
            constexpr auto LOG_TASK_STACK_DEPTH = 1024UL;
            constexpr auto LOG_TASK_NAME = "log_task";
            constexpr auto LOG_TASK_ARG = nullptr;

            static auto static_log_task = StaticTask_t{};
            static auto log_task_stack = std::array<StackType_t, LOG_TASK_STACK_DEPTH>{};

            xTaskCreateStatic(&log_task,
                              LOG_TASK_NAME,
                              log_task_stack.size(),
                              LOG_TASK_ARG,
                              LOG_TASK_PRIORITY,
                              log_task_stack.data(),
                              &static_log_task);
        }

    }; // namespace

    void log_manager_init() noexcept
    {
        log_queue_init();
        log_task_init();
    }

}; // namespace segway