#include "control_manager.hpp"
#include "FreeRTOS.h"
#include "log.hpp"
#include "pid.hpp"
#include "queue.h"
#include "queue_manager.hpp"
#include "task.h"
#include "task_manager.hpp"
#include <array>

namespace segway {

    namespace {

        constexpr auto TAG = "control_manager";

        constexpr auto PID_Y_REF = 2.75F64;
        constexpr auto PID_KP = 15.0F64;
        constexpr auto PID_KI = 0.0F64;
        constexpr auto PID_KD = 0.0F64;
        constexpr auto PID_KC = 0.0F64;
        constexpr auto PID_TD = 0.0001F64;
        constexpr auto PID_SAT = 1000.0F64;

        constexpr auto TILT_FAULT_THRESH_LOW = 45.0F64;
        constexpr auto TILT_FAULT_THRESH_HIGH = 30.0F64;

        struct Context {
            utility::PID<std::float64_t> regulator;

            struct Config {
                std::float64_t tilt_fault_thresh_low;
                std::float64_t tilt_fault_thresh_high;
            } config;
        } ctx;

        void process_imu_data(ControlEventPayload const& payload) noexcept
        {
            LOG(TAG, "process_imu_data");

            auto tilt = payload.imu_data.roll;
            auto error_tilt = PID_Y_REF - tilt;
            auto speed = ctx.regulator(error_tilt, payload.imu_data.dt);

            auto control_queue = get_control_queue();
            auto event = WheelEvent{.type = WheelEventType::WHEEL_DATA};
            event.payload.wheel_data = {.left_wheel_speed = speed,
                                        .right_wheel_speed = -speed,
                                        .dt = payload.imu_data.dt};

            xQueueSend(control_queue, &event, pdMS_TO_TICKS(10));
        }

        void process_control_queue_events() noexcept
        {
            LOG(TAG, "process_control_queue_events");

            auto event = ControlEvent{};
            auto control_queue = get_control_queue();

            while (uxQueueMessagesWaiting(control_queue)) {
                if (xQueueReceive(control_queue, &event, pdMS_TO_TICKS(10))) {
                    switch (event.type) {
                        case ControlEventType::IMU_DATA:
                            process_imu_data(event.payload);
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        void control_task(void*) noexcept
        {
            LOG(TAG, "control_task start");

            while (1) {
                process_control_queue_events();
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            LOG(TAG, "control_task end");
        }

        inline void control_task_init() noexcept
        {
            constexpr auto CONTROL_TASK_PRIORITY = 1UL;
            constexpr auto CONTROL_TASK_STACK_DEPTH = 1024UL;
            constexpr auto CONTROL_TASK_NAME = "control_task";
            constexpr auto CONTROL_TASK_ARG = nullptr;

            static auto static_control_task = StaticTask_t{};
            static auto control_task_stack = std::array<StackType_t, CONTROL_TASK_STACK_DEPTH>{};

            set_control_task(xTaskCreateStatic(&control_task,
                                               CONTROL_TASK_NAME,
                                               control_task_stack.size(),
                                               CONTROL_TASK_ARG,
                                               CONTROL_TASK_PRIORITY,
                                               control_task_stack.data(),
                                               &static_control_task));
        }

        inline void control_queue_init() noexcept
        {
            constexpr auto CONTROL_QUEUE_ITEM_SIZE = sizeof(ControlEvent);
            constexpr auto CONTROL_QUEUE_ITEMS = 10UL;
            constexpr auto CONTROL_QUEUE_STORAGE_SIZE =
                CONTROL_QUEUE_ITEM_SIZE * CONTROL_QUEUE_ITEMS;

            static auto static_control_queue = StaticQueue_t{};
            static auto control_queue_storage =
                std::array<std::uint8_t, CONTROL_QUEUE_STORAGE_SIZE>{};

            set_control_queue(xQueueCreateStatic(CONTROL_QUEUE_ITEMS,
                                                 CONTROL_QUEUE_ITEM_SIZE,
                                                 control_queue_storage.data(),
                                                 &static_control_queue));
        }

    }; // namespace

    void control_manager_init() noexcept
    {
        LOG(TAG, "manager_init");

        ctx.regulator.kP = PID_KP;
        ctx.regulator.kI = PID_KI;
        ctx.regulator.kD = PID_KD;
        ctx.regulator.kC = PID_KC;
        ctx.regulator.sat = PID_SAT;

        control_queue_init();
        control_task_init();
    }

}; // namespace segway