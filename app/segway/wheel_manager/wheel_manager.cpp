#include "wheel_manager.hpp"
#include "FreeRTOS.h"
#include "event_group_manager.hpp"
#include "event_groups.h"
#include "gpio.h"
#include "log.hpp"
#include "message_buffer_manager.hpp"
#include "queue.h"
#include "queue_manager.hpp"
#include "task.h"
#include "task_manager.hpp"
#include "tim.h"
#include "utility.hpp"
#include "wheel.hpp"
#include <cassert>

namespace segway {

    namespace {

        constexpr auto TAG = "wheel_manager";

        struct WheelManagerCtx {
            struct {
                std::array<Wheel, 2U> wheels;
                bool has_started;
            } periph;

            struct {
                std::float64_t fault_thresh_low;
                std::float64_t fault_thresh_high;
                std::float64_t wheel_distance;
            } config;

            bool is_running;
        } ctx;

        WheelDriver& get_wheel_driver(WheelType const type) noexcept
        {
            auto* it = std::find_if(ctx.periph.wheels.begin(),
                                    ctx.periph.wheels.end(),
                                    [type](Wheel const& wheel) { return wheel.type == type; });

            assert(it != ctx.periph.wheels.end());
            return it->driver;
        }

        inline bool receive_wheel_event(WheelEvent& event) noexcept
        {
#ifdef USE_QUEUES
            return xQueueReceive(get_queue(QueueType::WHEEL), &event, pdMS_TO_TICKS(10));
#else
            return xMessageBufferReceive(get_message_buffer(MessageBufferType::WHEEL),
                                         &event,
                                         sizeof(event),
                                         pdMS_TO_TICKS(10)) == sizeof(event);
#endif
        }

        inline EventBits_t wait_wheel_event_bits() noexcept
        {
#ifdef USE_EVENT_GROUPS
            return xEventGroupWaitBits(get_event_group(EventGroupType::WHEEL),
                                       WheelEventBit::ALL,
                                       pdTRUE,
                                       pdFALSE,
                                       pdMS_TO_TICKS(10));
#else
            auto event_bits = 0UL;
            xTaskNotifyWait(0x0000, WheelEventBit::ALL, &event_bits, pdMS_TO_TICKS(10));
            return event_bits;
#endif
        }

        void start_wheels() noexcept
        {
            LOG(TAG, "start_wheels!");

            for (auto& [type, driver] : ctx.periph.wheels) {
                driver.start();
            }
        }

        void stop_wheels() noexcept
        {
            LOG(TAG, "stop_wheels!");

            for (auto& [type, driver] : ctx.periph.wheels) {
                driver.stop();
            }
        }

        void set_wheels_speed(std::float64_t const left_speed,
                              std::float64_t const right_speed,
                              std::float64_t const dt) noexcept
        {
            LOG(TAG, "L wheel speed: %f, R wheel speed: %f, dt: %f", left_speed, right_speed, dt);

            get_wheel_driver(WheelType::LEFT).set_wheel_speed(left_speed, dt);
            get_wheel_driver(WheelType::RIGHT).set_wheel_speed(right_speed, dt);
        }

        void process_control_data(WheelEventPayload const& payload) noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_control_data");

            // if (!ctx.periph.has_started && payload.control_data.should_run) {
            //     ctx.periph_has_started = true;
            //     start_wheels();
            // } else if (ctx.periph.has_started && !payload.control_data.should_run) {
            //     ctx.periph.has_started = false;
            //     stop_wheels();
            //     return;
            // }

            set_wheels_speed(payload.control_data.left_speed,
                             payload.control_data.right_speed,
                             payload.control_data.dt);
        }

        void process_wheel_queue_events() noexcept
        {
            LOG(TAG, "process_wheel_queue_events");

            auto event = WheelEvent{};
            while (receive_wheel_event(event)) {
                switch (event.type) {
                    case WheelEventType::CONTROL_DATA:
                        process_control_data(event.payload);
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

            ctx.is_running = true;
            // HAL_TIM_Base_Start_IT(&htim1);
            // HAL_TIM_Base_Start_IT(&htim3);
        }

        void process_stop() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            ctx.is_running = false;

            LOG(TAG, "process_stop");

            // HAL_TIM_Base_Stop_IT(&htim1);
            // HAL_TIM_Base_Stop_IT(&htim3);
        }

        void process_left_step_timer() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_left_step_timer");

            get_wheel_driver(WheelType::LEFT).update_step_count();

            HAL_GPIO_TogglePin(GPIOA, 1 << 6);
            //   HAL_TIM_Base_Start_IT(&htim1);
        };

        void process_right_step_timer() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_right_step_timer");

            get_wheel_driver(WheelType::RIGHT).update_step_count();

            HAL_GPIO_TogglePin(GPIOA, 1 << 8);
            // HAL_TIM_Base_Start_IT(&htim3);
        };

        void process_wheel_event_group_bits() noexcept
        {
            LOG(TAG, "process_event_group_bits");

            auto event_bits = wait_wheel_event_bits();

            if ((event_bits & WheelEventBit::START) == WheelEventBit::START) {
                process_start();
            }

            if ((event_bits & WheelEventBit::STOP) == WheelEventBit::STOP) {
                process_stop();
            }

            if ((event_bits & WheelEventBit::LEFT_STEP_TIMER) == WheelEventBit::LEFT_STEP_TIMER) {
                process_left_step_timer();
            }

            if ((event_bits & WheelEventBit::RIGHT_STEP_TIMER) == WheelEventBit::RIGHT_STEP_TIMER) {
                process_right_step_timer();
            }
        };

        void wheel_task(void*) noexcept
        {
            LOG(TAG, "wheel_task start");

            while (1) {
                process_wheel_queue_events();
                process_wheel_event_group_bits();
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            LOG(TAG, "wheel_task end");
        }

        inline void wheel_task_init() noexcept
        {
            constexpr auto WHEEL_TASK_PRIORITY = 1UL;
            constexpr auto WHEEL_TASK_STACK_DEPTH = 1024UL;
            constexpr auto WHEEL_TASK_NAME = "wheel_task";
            constexpr auto WHEEL_TASK_ARG = nullptr;

            static auto wheel_static_task = StaticTask_t{};
            static auto wheel_task_stack = std::array<StackType_t, WHEEL_TASK_STACK_DEPTH>{};

            set_task(TaskType::WHEEL,
                     xTaskCreateStatic(&wheel_task,
                                       WHEEL_TASK_NAME,
                                       wheel_task_stack.size(),
                                       WHEEL_TASK_ARG,
                                       WHEEL_TASK_PRIORITY,
                                       wheel_task_stack.data(),
                                       &wheel_static_task));
        }

        inline void wheel_queue_init() noexcept
        {
#ifdef USE_QUEUES
            constexpr auto WHEEL_QUEUE_ITEM_SIZE = sizeof(WheelEvent);
            constexpr auto WHEEL_QUEUE_ITEMS = 10UL;
            constexpr auto WHEEL_QUEUE_STORAGE_SIZE = WHEEL_QUEUE_ITEM_SIZE * WHEEL_QUEUE_ITEMS;

            static auto wheel_static_queue = StaticQueue_t{};
            static auto wheel_queue_storage = std::array<std::uint8_t, WHEEL_QUEUE_STORAGE_SIZE>{};

            set_queue(QueueType::WHEEL,
                      xQueueCreateStatic(WHEEL_QUEUE_ITEMS,
                                         WHEEL_QUEUE_ITEM_SIZE,
                                         wheel_queue_storage.data(),
                                         &wheel_static_queue));
#else
            constexpr auto WHEEL_MESSAGE_BUFFER_ITEM_SIZE = sizeof(ControlEvent);
            constexpr auto WHEEL_MESSAGE_BUFFER_ITEMS = 10UL;
            constexpr auto WHEEL_MESSAGE_BUFFER_STORAGE_SIZE =
                WHEEL_MESSAGE_BUFFER_ITEM_SIZE * WHEEL_MESSAGE_BUFFER_ITEMS;

            static auto wheel_static_message_buffer = StaticMessageBuffer_t{};
            static auto wheel_message_buffer_storage =
                std::array<std::uint8_t, WHEEL_MESSAGE_BUFFER_STORAGE_SIZE>{};

            set_message_buffer(MessageBufferType::WHEEL,
                               xMessageBufferCreateStatic(wheel_message_buffer_storage.size(),
                                                          wheel_message_buffer_storage.data(),
                                                          &wheel_static_message_buffer));
#endif
        }

        inline void wheel_event_group_init() noexcept
        {
#ifdef USE_EVENT_GROUPS
            static auto wheel_static_event_group = StaticEventGroup_t{};

            set_event_group(EventGroupType::WHEEL,
                            xEventGroupCreateStatic(&wheel_static_event_group));
#endif
        }

        inline void wheel_periph_init() noexcept
        {
            constexpr auto MS1_1 = 1 << 11;
            constexpr auto MS2_1 = 1 << 10;
            constexpr auto MS3_1 = 1 << 9;
            constexpr auto DIR_1 = 1 << 15;
            constexpr auto EN_1 = 1 << 0;
            constexpr auto SLEEP_1 = 1 << 0;
            constexpr auto RESET_1 = 1 << 0;

            constexpr auto MS1_2 = 1 << 3;
            constexpr auto MS2_2 = 1 << 4;
            constexpr auto MS3_2 = 1 << 5;
            constexpr auto DIR_2 = 1 << 7;
            constexpr auto EN_2 = 1 << 0;
            constexpr auto SLEEP_2 = 1 << 0;
            constexpr auto RESET_2 = 1 << 0;

            constexpr auto STEPS_PER_360 = 200U;
            constexpr auto WHEEL_RADIUS = 1.0F64;

            auto a4988_1_config = a4988::Config{.pin_ms1 = MS1_1,
                                                .pin_ms2 = MS2_1,
                                                .pin_ms3 = MS3_1,
                                                .pin_reset = RESET_1,
                                                .pin_sleep = SLEEP_1,
                                                .pin_dir = DIR_1,
                                                .pin_enable = EN_1};
            auto a4988_2_config = a4988::Config{.pin_ms1 = MS1_2,
                                                .pin_ms2 = MS2_2,
                                                .pin_ms3 = MS3_2,
                                                .pin_reset = RESET_2,
                                                .pin_sleep = SLEEP_2,
                                                .pin_dir = DIR_2,
                                                .pin_enable = EN_2};

            auto a4988_gpio_write_pin = [](void* user, std::uint16_t pin, bool state) {
                auto port = static_cast<GPIO_TypeDef*>(user);
                HAL_GPIO_WritePin(port, pin, static_cast<GPIO_PinState>(state));
            };

            auto a4988_pulse_start = [](void* user) {
                auto handle = static_cast<TIM_HandleTypeDef*>(user);
                assert(HAL_OK == HAL_TIM_Base_Start_IT(handle));
            };
            auto a4988_pulse_stop = [](void* user) {
                auto handle = static_cast<TIM_HandleTypeDef*>(user);
                assert(HAL_OK == HAL_TIM_Base_Stop_IT(handle));
            };
            auto a4988_pulse_set_freq = [](void* user, std::uint32_t freq) {
                if (freq > 0UL) {
                    auto handle = static_cast<TIM_HandleTypeDef*>(user);
                    auto prescaler = 0UL;
                    auto period = 0UL;
                    utility::frequency_to_prescaler_and_period(freq,
                                                               84000000,
                                                               0,
                                                               0xFFFF,
                                                               0xFFFF,
                                                               prescaler,
                                                               period);
                    handle->Instance->PSC = prescaler;
                    handle->Instance->ARR = period;
                }
            };

            auto a4988_1_interface = a4988::Interface{.gpio_user = GPIOA,
                                                      .gpio_write_pin = a4988_gpio_write_pin,
                                                      .pulse_user = &htim1,
                                                      .pulse_start = a4988_pulse_start,
                                                      .pulse_stop = a4988_pulse_stop,
                                                      .pulse_set_freq = a4988_pulse_set_freq};
            auto a4988_2_interface = a4988::Interface{.gpio_user = GPIOA,
                                                      .gpio_write_pin = a4988_gpio_write_pin,
                                                      .pulse_user = &htim3,
                                                      .pulse_start = a4988_pulse_start,
                                                      .pulse_stop = a4988_pulse_stop,
                                                      .pulse_set_freq = a4988_pulse_set_freq};

            auto a4988_1 = a4988::A4988{.config = std::move(a4988_1_config),
                                        .interface = std::move(a4988_1_interface)};
            auto a4988_2 = a4988::A4988{.config = std::move(a4988_2_config),
                                        .interface = std::move(a4988_2_interface)};

            auto step_driver_1 = step_driver::StepDriver{.driver = std::move(a4988_1),
                                                         .steps_per_360 = STEPS_PER_360};
            auto step_driver_2 = step_driver::StepDriver{.driver = std::move(a4988_2),
                                                         .steps_per_360 = STEPS_PER_360};

            auto left_wheel =
                segway::Wheel{.type = segway::WheelType::LEFT,
                              .driver = segway::WheelDriver{.driver = std::move(step_driver_1),
                                                            .wheel_radius = WHEEL_RADIUS}};
            auto right_wheel =
                segway::Wheel{.type = segway::WheelType::RIGHT,
                              .driver = segway::WheelDriver{.driver = std::move(step_driver_2),
                                                            .wheel_radius = WHEEL_RADIUS}};

            left_wheel.driver.initialize();
            right_wheel.driver.initialize();

            ctx.periph.wheels[0] = std::move(left_wheel);
            ctx.periph.wheels[1] = std::move(right_wheel);
        }

        inline void wheel_config_init() noexcept
        {
            constexpr auto WHEEL_DIST = 10.0F64;
            constexpr auto FAULT_THRESH_HIGH = 1000.0F64;
            constexpr auto FAULT_THRESH_LOW = 800.0F64;

            ctx.config.fault_thresh_high = FAULT_THRESH_HIGH;
            ctx.config.fault_thresh_low = FAULT_THRESH_LOW;
            ctx.config.wheel_distance = WHEEL_DIST;

            ctx.is_running = false;
        }

    }; // namespace

    void wheel_manager_init() noexcept
    {
        std::memset(&ctx, 0, sizeof(ctx));

        wheel_periph_init();
        wheel_config_init();
        wheel_queue_init();
        wheel_event_group_init();
        wheel_task_init();
    }

}; // namespace segway
