#include "log_manager.hpp"
#include "FreeRTOS.h"
#include "log.hpp"
#include "queue_manager.hpp"
#include "semphr.h"
#include "task.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <array>
#include <cstring>

namespace segway {

    namespace {

        constexpr auto TAG = "log_manager";

        void log_task(void*) noexcept
        {
            auto event = LogEvent{};

            LOG(TAG, "log_task start");

            while (1) {
                while (xQueueReceive(get_queue(QueueType::LOG), &event, portMAX_DELAY)) {
                    auto msg = reinterpret_cast<std::uint8_t*>(event.buf);
                    auto msg_len = std::strlen(event.buf);

                    HAL_UART_Transmit(&huart2, msg, msg_len, HAL_MAX_DELAY);
                    // CDC_Transmit_FS(msg, msg_len);
                }

                vTaskDelay(pdMS_TO_TICKS(100));
            }

            LOG(TAG, "log_task end");
        }

        inline void log_queue_init() noexcept
        {
#ifdef DEBUG
            constexpr auto LOG_QUEUE_ITEM_SIZE = sizeof(LogEvent);
            constexpr auto LOG_QUEUE_ITEMS = 10UL;
            constexpr auto LOG_QUEUE_STORAGE_SIZE = LOG_QUEUE_ITEM_SIZE * LOG_QUEUE_ITEMS;

            static auto log_static_queue = StaticQueue_t{};
            static auto log_queue_storage = std::array<std::uint8_t, LOG_QUEUE_STORAGE_SIZE>{};

            set_queue(QueueType::LOG,
                      xQueueCreateStatic(LOG_QUEUE_ITEMS,
                                         LOG_QUEUE_ITEM_SIZE,
                                         log_queue_storage.data(),
                                         &log_static_queue));
#endif
        }

        inline void log_task_init() noexcept
        {
#ifdef DEBUG
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
#endif
        }

    }; // namespace

    void log_manager_init() noexcept
    {
#ifdef DEBUG
        log_queue_init();
        log_task_init();
#endif
    }

}; // namespace segway