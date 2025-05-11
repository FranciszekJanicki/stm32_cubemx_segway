#include "wheel_manager.hpp"
#include "FreeRTOS.h"
#include "event_bits.hpp"
#include "event_group_manager.hpp"
#include "event_groups.h"
#include "events.hpp"
#include "gpio.h"
#include "gpio.hpp"
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
                std::array<WheelDriver, 2U> wheel_drivers;
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
            // auto* it = std::find_if(ctx.periph.wheels.begin(),
            //                         ctx.periph.wheels.end(),
            //                         [type](Wheel const& wheel) { return wheel.type == type; });

            return ctx.periph.wheel_drivers[std::to_underlying(type)];
        }

        inline bool receive_wheel_event(WheelEvent& event) noexcept
        {
#ifdef USE_QUEUES
            return xQueueReceive(get_queue(QueueType::WHEEL), &event, pdMS_TO_TICKS(1));
#else
            return xMessageBufferReceive(get_message_buffer(MessageBufferType::WHEEL),
                                         &event,
                                         sizeof(event),
                                         pdMS_TO_TICKS(1)) == sizeof(event);
#endif
        }

        inline std::uint32_t wait_wheel_event_bits() noexcept
        {
#ifdef USE_EVENT_GROUPS
            return xEventGroupWaitBits(get_event_group(EventGroupType::WHEEL),
                                       WheelEventBit::ALL,
                                       pdTRUE,
                                       pdFALSE,
                                       pdMS_TO_TICKS(1));
#else
            auto event_bits = 0UL;
            xTaskNotifyWait(0x00, WheelEventBit::ALL, &event_bits, pdMS_TO_TICKS(1));
            return event_bits;
#endif
        }

        void set_wheels_speed(std::float64_t const left_speed,
                              std::float64_t const right_speed,
                              std::float64_t const dt) noexcept
        {
            LOG(TAG, "L wheel speed: %f, R wheel speed: %f, dt: %f", left_speed, right_speed, dt);

            get_wheel_driver(WheelType::WHEEL_LEFT).set_wheel_speed(left_speed, dt);
            get_wheel_driver(WheelType::WHEEL_RIGHT).set_wheel_speed(right_speed, dt);
        }

        void process_control_data(WheelEventPayload const& payload) noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_control_data");

            // if (!ctx.periph.has_started && payload.control_data.should_run) {
            //     ctx.periph_has_started = true;
            //     for (auto& wheel_driver : ctx.periph.wheel_drivers) {
            //         wheel_driver.start();
            //     }
            // } else if (ctx.periph.has_started && !payload.control_data.should_run) {
            //     ctx.periph.has_started = false;
            //     for (auto& wheel_driver : ctx.periph.wheel_drivers) {
            //         wheel_driver.stop();
            //     }
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
            if (receive_wheel_event(event)) {
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
            HAL_TIM_Base_Start_IT(&htim1);
            HAL_TIM_Base_Start_IT(&htim3);
        }

        void process_stop() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            ctx.is_running = false;

            LOG(TAG, "process_stop");

            HAL_TIM_Base_Stop_IT(&htim1);
            HAL_TIM_Base_Stop_IT(&htim3);
        }

        void process_left_step_timer() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_left_step_timer");

            get_wheel_driver(WheelType::WHEEL_LEFT).update_step_count();

            // HAL_GPIO_TogglePin(GPIOA, 1 << 8);
            HAL_TIM_Base_Start_IT(&htim1);
        };

        void process_right_step_timer() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_right_step_timer");

            get_wheel_driver(WheelType::WHEEL_RIGHT).update_step_count();

            //   HAL_GPIO_TogglePin(GPIOA, 1 << 6);
            HAL_TIM_Base_Start_IT(&htim3);
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
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            LOG(TAG, "wheel_task end");
        }

        inline void wheel_task_init() noexcept
        {
            constexpr auto WHEEL_TASK_PRIORITY = 2UL;
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
            constexpr auto WHEEL_QUEUE_ITEMS = 50UL;
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
            constexpr auto WHEEL_MESSAGE_BUFFER_ITEMS = 50UL;
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
            using namespace hal;

            constexpr auto MS1_1 = std::to_underlying(GPIOPin::NC);
            constexpr auto MS2_1 = std::to_underlying(GPIOPin::NC);
            constexpr auto MS3_1 = std::to_underlying(GPIOPin::NC);
            constexpr auto DIR_1 = std::to_underlying(GPIOPin::PB15);
            constexpr auto EN_1 = std::to_underlying(GPIOPin::NC);
            constexpr auto SLEEP_1 = std::to_underlying(GPIOPin::NC);
            constexpr auto RESET_1 = std::to_underlying(GPIOPin::NC);

            constexpr auto MS1_2 = std::to_underlying(GPIOPin::NC);
            constexpr auto MS2_2 = std::to_underlying(GPIOPin::NC);
            constexpr auto MS3_2 = std::to_underlying(GPIOPin::NC);
            constexpr auto DIR_2 = std::to_underlying(GPIOPin::PA7);
            constexpr auto EN_2 = std::to_underlying(GPIOPin::NC);
            constexpr auto SLEEP_2 = std::to_underlying(GPIOPin::NC);
            constexpr auto RESET_2 = std::to_underlying(GPIOPin::NC);

            constexpr auto STEPS_PER_360 = 200U;
            constexpr auto WHEEL_RADIUS = 1.0F64;

            auto a4988_gpio_write_pin = [](void* user, std::uint16_t pin, bool state) {
                gpio_write_pin(static_cast<GPIOPin>(pin), static_cast<GPIOState>(state));
            };

            auto a4988_pulse_set_freq = [](void* user, std::uint32_t freq) {
                if (freq > 0UL) {
                    auto handle = static_cast<TIM_HandleTypeDef*>(user);
                    auto psc = 0UL;
                    auto period = 0UL;
                    utility::frequency_to_prescaler_and_period(2 * freq,
                                                               84000000,
                                                               0,
                                                               0xFFFF,
                                                               0xFFFF,
                                                               psc,
                                                               period);
                    LOG(TAG, "Frequency: %ld, prescaler: %ld, period: %ld", freq, psc, period);
                    handle->Instance->PSC = psc;
                    handle->Instance->ARR = period;
                }
            };

            auto a4988_configs = std::array<a4988::Config, 2UL>{};

            a4988_configs[WheelType::WHEEL_LEFT] = {.pin_ms1 = MS1_1,
                                                    .pin_ms2 = MS2_1,
                                                    .pin_ms3 = MS3_1,
                                                    .pin_reset = RESET_1,
                                                    .pin_sleep = SLEEP_1,
                                                    .pin_dir = DIR_1,
                                                    .pin_enable = EN_1};

            a4988_configs[WheelType::WHEEL_RIGHT] = {.pin_ms1 = MS1_2,
                                                     .pin_ms2 = MS2_2,
                                                     .pin_ms3 = MS3_2,
                                                     .pin_reset = RESET_2,
                                                     .pin_sleep = SLEEP_2,
                                                     .pin_dir = DIR_2,
                                                     .pin_enable = EN_2};

            auto a4988_interfaces = std::array<a4988::Interface, 2UL>{};

            a4988_interfaces[WheelType::WHEEL_LEFT] = {.gpio_user = nullptr,
                                                       .gpio_init = nullptr,
                                                       .gpio_deinit = nullptr,
                                                       .gpio_write_pin = a4988_gpio_write_pin,
                                                       .pulse_user = &htim1,
                                                       .pulse_init = nullptr,
                                                       .pulse_deinit = nullptr,
                                                       .pulse_start = nullptr,
                                                       .pulse_stop = nullptr,
                                                       .pulse_set_freq = a4988_pulse_set_freq};

            a4988_interfaces[WheelType::WHEEL_RIGHT] = {.gpio_user = nullptr,
                                                        .gpio_init = nullptr,
                                                        .gpio_deinit = nullptr,
                                                        .gpio_write_pin = a4988_gpio_write_pin,
                                                        .pulse_user = &htim3,
                                                        .pulse_init = nullptr,
                                                        .pulse_deinit = nullptr,
                                                        .pulse_start = nullptr,
                                                        .pulse_stop = nullptr,
                                                        .pulse_set_freq = a4988_pulse_set_freq};

            auto a4988s = std::array<a4988::A4988, 2UL>{};

            a4988s[WheelType::WHEEL_LEFT] = {.config =
                                                 std::move(a4988_configs[WheelType::WHEEL_LEFT]),
                                             .interface = std::move(
                                                 a4988_interfaces[WheelType::WHEEL_LEFT])},

            a4988s[WheelType::WHEEL_RIGHT] = {
                .config = std::move(a4988_configs[WheelType::WHEEL_RIGHT]),
                .interface = std::move(a4988_interfaces[WheelType::WHEEL_RIGHT])};

            auto step_drivers = std::array<step_driver::StepDriver, 2UL>{};

            step_drivers[WheelType::WHEEL_LEFT] = {.driver =
                                                       std::move(a4988s[WheelType::WHEEL_LEFT]),
                                                   .steps_per_360 = STEPS_PER_360};

            step_drivers[WheelType::WHEEL_RIGHT] = {.driver =
                                                        std::move(a4988s[WheelType::WHEEL_RIGHT]),
                                                    .steps_per_360 = STEPS_PER_360};

            ctx.periph.wheel_drivers[WheelType::WHEEL_LEFT] =
                WheelDriver{.driver = std::move(step_drivers[WheelType::WHEEL_LEFT]),
                            .wheel_radius = WHEEL_RADIUS};

            ctx.periph.wheel_drivers[WheelType::WHEEL_RIGHT] =
                WheelDriver{.driver = std::move(step_drivers[WheelType::WHEEL_RIGHT]),
                            .wheel_radius = WHEEL_RADIUS};

            for (auto& wheel_driver : ctx.periph.wheel_drivers) {
                wheel_driver.initialize();
            }
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
