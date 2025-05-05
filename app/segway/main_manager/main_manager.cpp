#include "main_manager.hpp"
#include "FreeRTOS.h"
#include "event_group_manager.hpp"
#include "event_groups.h"
#include "log.hpp"
#include "task.h"
#include "task_manager.hpp"
#include "tim.h"
#include <cassert>

namespace segway {

    namespace {

        constexpr auto TAG = "main_manager";

        inline void set_imu_event_bits(EventBits_t const event_bits) noexcept
        {
#ifdef USE_EVENT_GROUPS
            xEventGroupSetBits(get_event_group(EventGroupType::IMU), event_bits);
#else
            xTaskNotify(get_task(TaskType::IMU), event_bits, eNotifyAction::eSetBits);
#endif
        }

        void main_task(void*) noexcept
        {
            LOG(TAG, "main_task start");

            set_imu_event_bits(IMUEventBit::START);

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
