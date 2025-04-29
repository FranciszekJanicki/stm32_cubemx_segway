#include "imu_task.hpp"
#include "FreeRTOS.h"
#include "imu_manager.hpp"
#include "log.hpp"
#include "task.h"
#include "task_manager.hpp"
#include <array>

namespace segway {

    namespace {

        constexpr auto TAG = "imu_task";

        inline auto imu_static_task = StaticTask_t{};
        inline auto imu_task_stack = std::array<StackType_t, IMU_TASK_STACK_DEPTH>{};

        void imu_task(void*) noexcept
        {
            LOG(TAG, "imu_task start");

            //   imu_manager_init();

            while (1) {
                imu_manager_process();
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            LOG(TAG, "imu_task end");
        }

    }; // namespace

    void imu_task_init() noexcept
    {
        auto task = xTaskCreateStatic(&imu_task,
                                      IMU_TASK_NAME,
                                      imu_task_stack.size(),
                                      IMU_TASK_ARG,
                                      IMU_TASK_PRIORITY,
                                      imu_task_stack.data(),
                                      &imu_static_task);

        set_task(TaskType::IMU, task);
    }

}; // namespace segway