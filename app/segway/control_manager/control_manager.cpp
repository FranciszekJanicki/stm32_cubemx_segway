#include "control_manager.hpp"
#include "FreeRTOS.h"
#include "event_group_manager.hpp"
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

        struct Context {
            struct {
                utility::PID<std::float64_t> pid;
            } regulator;

            struct Config {
                std::float64_t tilt_fault_thresh_low;
                std::float64_t tilt_fault_thresh_high;
                std::float64_t tilt_ref;
            } config;

            bool is_running;
        } ctx;

        void process_start() noexcept
        {
            LOG(TAG, "process_start");

            if (!ctx.is_running) {
                ctx.is_running = true;

                auto event = WheelEvent{.type = WheelEventType::START};
                xQueueSend(get_queue(QueueType::WHEEL), &event, pdMS_TO_TICKS(10));
            }
        }

        void process_stop() noexcept
        {
            LOG(TAG, "process_stop");

            if (ctx.is_running) {
                ctx.is_running = false;

                auto event = WheelEvent{.type = WheelEventType::STOP};
                xQueueSend(get_queue(QueueType::WHEEL), &event, pdMS_TO_TICKS(10));
            }
        }

        void process_imu_data(ControlEventPayload const& payload) noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_imu_data");

            LOG(TAG,
                "roll: %f, pitch: %f, yaw: %f, dt: %f",
                payload.imu_data.roll,
                payload.imu_data.pitch,
                payload.imu_data.yaw,
                payload.imu_data.dt);

            auto tilt = payload.imu_data.roll;
            auto error_tilt = ctx.config.tilt_ref - tilt;
            auto speed = ctx.regulator.pid.get_sat_u(error_tilt, payload.imu_data.dt);

            auto event = WheelEvent{.type = WheelEventType::WHEEL_DATA};
            event.payload.wheel_data = {.left_speed = speed,
                                        .right_speed = -speed,
                                        .dt = payload.imu_data.dt};

            if (!xQueueSend(get_queue(QueueType::WHEEL), &event, pdMS_TO_TICKS(1))) {
                LOG(TAG, "Failed sending to queue!");
            }
        }

        void process_control_queue_events() noexcept
        {
            LOG(TAG, "process_control_queue_events");

            auto event = ControlEvent{};

            if (xQueueReceive(get_queue(QueueType::CONTROL), &event, pdMS_TO_TICKS(10))) {
                switch (event.type) {
                    case ControlEventType::START:
                        process_start();
                        break;
                    case ControlEventType::STOP:
                        process_stop();
                        break;
                    case ControlEventType::IMU_DATA:
                        process_imu_data(event.payload);
                        break;
                    default:
                        break;
                }
            }
        }

        void process_control_event_group_bits() noexcept
        {
            LOG(TAG, "process_control_event_group_bits");
        }

        void control_task(void*) noexcept
        {
            LOG(TAG, "control_task start");

            while (1) {
                process_control_queue_events();
                process_control_event_group_bits();
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

            static auto control_static_task = StaticTask_t{};
            static auto control_task_stack = std::array<StackType_t, CONTROL_TASK_STACK_DEPTH>{};

            set_task(TaskType::CONTROL,
                     xTaskCreateStatic(&control_task,
                                       CONTROL_TASK_NAME,
                                       control_task_stack.size(),
                                       CONTROL_TASK_ARG,
                                       CONTROL_TASK_PRIORITY,
                                       control_task_stack.data(),
                                       &control_static_task));
        }

        inline void control_queue_init() noexcept
        {
            constexpr auto CONTROL_QUEUE_ITEM_SIZE = sizeof(ControlEvent);
            constexpr auto CONTROL_QUEUE_ITEMS = 100UL;
            constexpr auto CONTROL_QUEUE_STORAGE_SIZE =
                CONTROL_QUEUE_ITEM_SIZE * CONTROL_QUEUE_ITEMS;

            static auto control_static_queue = StaticQueue_t{};
            static auto control_queue_storage =
                std::array<std::uint8_t, CONTROL_QUEUE_STORAGE_SIZE>{};

            set_queue(QueueType::CONTROL,
                      xQueueCreateStatic(CONTROL_QUEUE_ITEMS,
                                         CONTROL_QUEUE_ITEM_SIZE,
                                         control_queue_storage.data(),
                                         &control_static_queue));
        }

        inline void control_regulator_init() noexcept
        {
            constexpr auto PID_KP = 15.0F64;
            constexpr auto PID_KI = 0.0F64;
            constexpr auto PID_KD = 0.0F64;
            constexpr auto PID_KC = 0.0F64;
            constexpr auto PID_TD = 0.0001F64;
            constexpr auto PID_SAT = 1000.0F64;

            ctx.regulator.pid.kP = PID_KP;
            ctx.regulator.pid.kI = PID_KI;
            ctx.regulator.pid.kD = PID_KD;
            ctx.regulator.pid.kC = PID_KC;
            ctx.regulator.pid.sat = PID_SAT;
        }

        inline void control_config_init() noexcept
        {
            constexpr auto TILT_FAULT_THRESH_LOW = 45.0F64;
            constexpr auto TILT_FAULT_THRESH_HIGH = 30.0F64;

            ctx.config.tilt_fault_thresh_high = TILT_FAULT_THRESH_HIGH;
            ctx.config.tilt_fault_thresh_low = TILT_FAULT_THRESH_LOW;
            ctx.config.tilt_ref = 2.75F64;

            ctx.is_running = false;
        }

    }; // namespace

    void control_manager_init() noexcept
    {
        control_config_init();
        control_regulator_init();
        control_queue_init();
        control_task_init();
    }

}; // namespace segway