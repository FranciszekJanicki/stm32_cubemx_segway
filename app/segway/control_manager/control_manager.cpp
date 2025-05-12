#include "control_manager.hpp"
#include "FreeRTOS.h"
#include "event_bits.hpp"
#include "event_group_manager.hpp"
#include "events.hpp"
#include "log.hpp"
#include "message_buffer_manager.hpp"
#include "pid.hpp"
#include "queue.h"
#include "queue_manager.hpp"
#include "task.h"
#include "task_manager.hpp"
#include <array>

namespace segway {

    namespace {

        constexpr auto TAG = "control_manager";

        struct ControlManagerCtx {
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

        inline bool send_wheel_event(WheelEvent const& event) noexcept
        {
#ifdef USE_QUEUES
            return xQueueOverwrite(get_queue(QueueType::WHEEL), &event);
#else
            return xMessageBufferSend(get_message_buffer(MessageBufferType::WHEEL),
                                      &event,
                                      sizeof(event),
                                      pdMS_TO_TICKS(1)) == sizeof(event);
#endif
        }

        inline bool receive_control_event(ControlEvent& event) noexcept
        {
#ifdef USE_QUEUES
            return xQueuePeek(get_queue(QueueType::CONTROL), &event, pdMS_TO_TICKS(1));
#else
            return xMessageBufferReceive(get_message_buffer(MessageBufferType::CONTROL),
                                         &event,
                                         sizeof(event),
                                         pdMS_TO_TICKS(1)) == sizeof(event);
#endif
        }

        inline std::uint32_t wait_control_event_bits() noexcept
        {
#ifdef USE_EVENT_GROUPS
            return xEventGroupWaitBits(get_event_group(EventGroupType::CONTROL),
                                       ControlEventBit::ALL,
                                       pdTRUE,
                                       pdFALSE,
                                       pdMS_TO_TICKS(1));
#else
            auto event_bits = 0UL;
            xTaskNotifyWait(0x00, ControlEventBit::ALL, &event_bits, pdMS_TO_TICKS(1));
            return event_bits;
#endif
        }

        inline void set_wheel_event_bits(std::uint32_t const event_bits) noexcept
        {
#ifdef USE_EVENT_GROUPS
            xEventGroupSetBits(get_event_group(EventGroupType::WHEEL), event_bits);
#else
            xTaskNotify(get_task(TaskType::WHEEL), event_bits, eNotifyAction::eSetBits);
#endif
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

            auto event = WheelEvent{.type = WheelEventType::CONTROL_DATA};

            auto tilt = payload.imu_data.roll;

            // if (std::abs(tilt) > ctx.config.tilt_fault_thresh_high ||
            //     std::abs(tilt) < ctx.config.tilt_fault_thresh_low) {
            //     event.payload.control_data.should_run = false;
            // } else {
            auto error_tilt = ctx.config.tilt_ref - tilt;
            auto speed = ctx.regulator.pid.get_sat_u(error_tilt, payload.imu_data.dt);

            event.payload.control_data.left_speed = -speed;
            event.payload.control_data.right_speed = speed;
            event.payload.control_data.dt = payload.imu_data.dt;
            event.payload.control_data.should_run = true;
            // }

            if (!send_wheel_event(event)) {
                LOG(TAG, "Failed send_wheel_event");
            }
        }

        void process_control_queue_events() noexcept
        {
            LOG(TAG, "process_control_queue_events");

            auto event = ControlEvent{};
            if (receive_control_event(event)) {
                switch (event.type) {
                    case ControlEventType::IMU_DATA:
                        process_imu_data(event.payload);
                        break;
                    default:
                        break;
                }
            }
        }

        void process_start() noexcept
        {
            if (ctx.is_running) {
                return;
            }

            LOG(TAG, "process_start");

            set_wheel_event_bits(WheelEventBit::START);
            ctx.is_running = true;
        }

        void process_stop() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_stop");

            set_wheel_event_bits(WheelEventBit::STOP);
            ctx.is_running = false;
        }

        void process_control_event_group_bits() noexcept
        {
            LOG(TAG, "process_control_event_group_bits");

            auto event_bits = wait_control_event_bits();

            if ((event_bits & ControlEventBit::START) == ControlEventBit::START) {
                process_start();
            }

            if ((event_bits & ControlEventBit::STOP) == ControlEventBit::STOP) {
                process_stop();
            }
        }

        void control_task(void*) noexcept
        {
            LOG(TAG, "control_task start");

            while (1) {
                process_control_queue_events();
                process_control_event_group_bits();
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            LOG(TAG, "control_task end");
        }

        inline void control_task_init() noexcept
        {
            constexpr auto CONTROL_TASK_PRIORITY = 2UL;
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

        inline void control_event_group_init() noexcept
        {
#ifdef USE_EVENT_GROUPS
            static auto control_static_event_group = StaticEventGroup_t{};

            set_event_group(EventGroupType::CONTROL,
                            xEventGroupCreateStatic(&control_static_event_group));
#endif
        }

        inline void control_queue_init() noexcept
        {
#ifdef USE_QUEUES
            constexpr auto CONTROL_QUEUE_ITEM_SIZE = sizeof(ControlEvent);
            constexpr auto CONTROL_QUEUE_ITEMS = 1UL;
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
#else
            constexpr auto CONTROL_MESSAGE_BUFFER_ITEM_SIZE = sizeof(ControlEvent);
            constexpr auto CONTROL_MESSAGE_BUFFER_ITEMS = 1UL;
            constexpr auto CONTROL_MESSAGE_BUFFER_STORAGE_SIZE =
                CONTROL_MESSAGE_BUFFER_ITEM_SIZE * CONTROL_MESSAGE_BUFFER_ITEMS;

            static auto control_static_message_buffer = StaticMessageBuffer_t{};
            static auto control_message_buffer_storage =
                std::array<std::uint8_t, CONTROL_MESSAGE_BUFFER_STORAGE_SIZE>{};

            set_message_buffer(MessageBufferType::CONTROL,
                               xMessageBufferCreateStatic(control_message_buffer_storage.size(),
                                                          control_message_buffer_storage.data(),
                                                          &control_static_message_buffer));
#endif
        }

        inline void control_regulator_init() noexcept
        {
            constexpr auto PID_KP = 75.0F64;
            constexpr auto PID_KI = 0.0F64;
            constexpr auto PID_KD = 0.0F64;
            constexpr auto PID_KC = 0.0F64;
            constexpr auto PID_TD = 0.0F64;
            constexpr auto PID_SAT = 100.0F64;

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
        std::memset(&ctx, 0, sizeof(ctx));

        control_config_init();
        control_regulator_init();
        control_queue_init();
        control_event_group_init();
        control_task_init();
    }

}; // namespace segway