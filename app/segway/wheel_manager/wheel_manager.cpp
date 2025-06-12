#include "wheel_manager.hpp"
#include "FreeRTOS.h"
#include "event_bits.hpp"
#include "event_group_manager.hpp"
#include "event_groups.h"
#include "events.hpp"
#include "gpio.h"
#include "gpio.hpp"
#include "log.hpp"
#include "message_buf_manager.hpp"
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
            std::array<WheelDriver, 2U> wheel_drivers;
            std::float64_t wheel_distance;
            bool has_started;
            bool is_running;
        } ctx;

        WheelDriver& get_wheel_driver(WheelType const type) noexcept
        {
            return ctx.wheel_drivers[std::to_underlying(type)];
        }

        inline bool receive_wheel_event(WheelEvent& event) noexcept
        {
#ifdef USE_QUEUES
            return xQueuePeek(get_queue(QueueType::WHEEL), &event, pdMS_TO_TICKS(1));
#else
            return xMessageBufferReceive(get_message_buf(MessageBufType::WHEEL),
                                         &event,
                                         sizeof(event),
                                         pdMS_TO_TICKS(1)) == sizeof(event);
#endif
        }

        inline std::uint32_t wait_wheel_event_bits() noexcept
        {
#ifdef USE_EVENT_GROUPS
            return xEventGroupWaitBits(get_event_group(EventGroupType::WHEEL),
                                       WHEEL_EVENT_BIT_ALL,
                                       pdTRUE,
                                       pdFALSE,
                                       pdMS_TO_TICKS(1));
#else
            auto event_bits = 0UL;
            xTaskNotifyWait(0x00, WHEEL_EVENT_BIT_ALL, &event_bits, pdMS_TO_TICKS(1));
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

            set_wheels_speed(payload.control_data.left_speed,
                             payload.control_data.right_speed,
                             payload.control_data.dt);
        }

        void process_wheel_queue_events() noexcept
        {
            LOG(TAG, "process_wheel_queue_events");

            WheelEvent event = {};
            if (receive_wheel_event(event)) {
                process_control_data(event.payload);
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

            HAL_GPIO_TogglePin(GPIOA, 1 << 8);
            HAL_TIM_Base_Start_IT(&htim1);
        };

        void process_right_step_timer() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_right_step_timer");

            get_wheel_driver(WheelType::WHEEL_RIGHT).update_step_count();

            HAL_GPIO_TogglePin(GPIOA, 1 << 6);
            HAL_TIM_Base_Start_IT(&htim3);
        };

        void process_wheel_event_group_bits() noexcept
        {
            LOG(TAG, "process_event_group_bits");

            auto event_bits = wait_wheel_event_bits();

            if ((event_bits & WHEEL_EVENT_BIT_START) == WHEEL_EVENT_BIT_START) {
                process_start();
            }

            if ((event_bits & WHEEL_EVENT_BIT_STOP) == WHEEL_EVENT_BIT_STOP) {
                process_stop();
            }

            if ((event_bits & WHEEL_EVENT_BIT_LEFT_STEP_TIMER) == WHEEL_EVENT_BIT_LEFT_STEP_TIMER) {
                process_left_step_timer();
            }

            if ((event_bits & WHEEL_EVENT_BIT_RIGHT_STEP_TIMER) ==
                WHEEL_EVENT_BIT_RIGHT_STEP_TIMER) {
                process_right_step_timer();
            }
        };

        void wheel_task(void*) noexcept
        {
            while (1) {
                process_wheel_queue_events();
                process_wheel_event_group_bits();
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        inline void wheel_task_init() noexcept
        {
            constexpr auto WHEEL_TASK_PRIORITY = 2UL;
            constexpr auto WHEEL_TASK_STACK_DEPTH = 1024UL;
            constexpr auto WHEEL_TASK_NAME = "wheel_task";
            constexpr auto WHEEL_TASK_ARG = nullptr;

            static StaticTask_t wheel_task_buffer = {};
            static std::array<StackType_t, WHEEL_TASK_STACK_DEPTH> wheel_task_stack = {};

            set_task(TaskType::WHEEL,
                     xTaskCreateStatic(&wheel_task,
                                       WHEEL_TASK_NAME,
                                       wheel_task_stack.size(),
                                       WHEEL_TASK_ARG,
                                       WHEEL_TASK_PRIORITY,
                                       wheel_task_stack.data(),
                                       &wheel_task_buffer));
        }

        inline void wheel_queue_init() noexcept
        {
#ifdef USE_QUEUES
            constexpr auto WHEEL_QUEUE_ITEM_SIZE = sizeof(WheelEvent);
            constexpr auto WHEEL_QUEUE_ITEMS = 1UL;
            constexpr auto WHEEL_QUEUE_STORAGE_SIZE = WHEEL_QUEUE_ITEM_SIZE * WHEEL_QUEUE_ITEMS;

            static StaticQueue_t wheel_queue_buffer = {};
            static std::array<std::uint8_t, WHEEL_QUEUE_STORAGE_SIZE> wheel_queue_storage = {};

            set_queue(QueueType::WHEEL,
                      xQueueCreateStatic(WHEEL_QUEUE_ITEMS,
                                         WHEEL_QUEUE_ITEM_SIZE,
                                         wheel_queue_storage.data(),
                                         &wheel_queue_buffer));
#else
            constexpr auto WHEEL_MESSAGE_BUFFER_ITEM_SIZE = sizeof(ControlEvent);
            constexpr auto WHEEL_MESSAGE_BUFFER_ITEMS = 1UL;
            constexpr auto WHEEL_MESSAGE_BUFFER_STORAGE_SIZE =
                WHEEL_MESSAGE_BUFFER_ITEM_SIZE * WHEEL_MESSAGE_BUFFER_ITEMS;

            static StaticMessageBuffer_t wheel_message_buf_buffer = {};
            static std::array<std::uint8_t, WHEEL_MESSAGE_BUFFER_STORAGE_SIZE>
                wheel_message_buf_storage = {};

            set_message_buf(MessageBufType::WHEEL,
                            xMessageBufferCreateStatic(wheel_message_buf_storage.size(),
                                                       wheel_message_buf_storage.data(),
                                                       &wheel_message_buf_buffer));
#endif
        }

        inline void wheel_event_group_init() noexcept
        {
#ifdef USE_EVENT_GROUPS
            static StaticEventGroup_t wheel_event_group_buffer = {};

            set_event_group(EventGroupType::WHEEL,
                            xEventGroupCreateStatic(&wheel_event_group_buffer));
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

            a4988::Config a4988_configs[] = {[WheelType::WHEEL_LEFT] = {.pin_ms1 = MS1_1,
                                                                        .pin_ms2 = MS2_1,
                                                                        .pin_ms3 = MS3_1,
                                                                        .pin_reset = RESET_1,
                                                                        .pin_sleep = SLEEP_1,
                                                                        .pin_dir = DIR_1,
                                                                        .pin_enable = EN_1},
                                             [WheelType::WHEEL_RIGHT] = {.pin_ms1 = MS1_2,
                                                                         .pin_ms2 = MS2_2,
                                                                         .pin_ms3 = MS3_2,
                                                                         .pin_reset = RESET_2,
                                                                         .pin_sleep = SLEEP_2,
                                                                         .pin_dir = DIR_2,
                                                                         .pin_enable = EN_2}};

            a4988::Interface a4988_interfaces[] = {
                [WheelType::WHEEL_LEFT] = {.gpio_user = nullptr,
                                           .gpio_init = nullptr,
                                           .gpio_deinit = nullptr,
                                           .gpio_write_pin = a4988_gpio_write_pin,
                                           .pulse_user = &htim1,
                                           .pulse_init = nullptr,
                                           .pulse_deinit = nullptr,
                                           .pulse_start = nullptr,
                                           .pulse_stop = nullptr,
                                           .pulse_set_freq = a4988_pulse_set_freq},
                [WheelType::WHEEL_RIGHT] = {.gpio_user = nullptr,
                                            .gpio_init = nullptr,
                                            .gpio_deinit = nullptr,
                                            .gpio_write_pin = a4988_gpio_write_pin,
                                            .pulse_user = &htim3,
                                            .pulse_init = nullptr,
                                            .pulse_deinit = nullptr,
                                            .pulse_start = nullptr,
                                            .pulse_stop = nullptr,
                                            .pulse_set_freq = a4988_pulse_set_freq}};

            a4988::A4988 a4988s[] = {
                [WheelType::WHEEL_LEFT] = {.config = a4988_configs[WheelType::WHEEL_LEFT],
                                           .interface = a4988_interfaces[WheelType::WHEEL_LEFT]},
                [WheelType::WHEEL_RIGHT] = {.config = a4988_configs[WheelType::WHEEL_RIGHT],
                                            .interface = a4988_interfaces[WheelType::WHEEL_RIGHT]}};

            step_driver::StepDriver step_drivers[] = {
                [WheelType::WHEEL_LEFT] = {.driver = a4988s[WheelType::WHEEL_LEFT],
                                           .steps_per_360 = STEPS_PER_360},
                [WheelType::WHEEL_RIGHT] = {.driver = a4988s[WheelType::WHEEL_RIGHT],
                                            .steps_per_360 = STEPS_PER_360}};

            ctx.wheel_drivers[WheelType::WHEEL_LEFT] =
                WheelDriver{.driver = step_drivers[WheelType::WHEEL_LEFT],
                            .wheel_radius = WHEEL_RADIUS};

            ctx.wheel_drivers[WheelType::WHEEL_RIGHT] =
                WheelDriver{.driver = step_drivers[WheelType::WHEEL_RIGHT],
                            .wheel_radius = WHEEL_RADIUS};

            for (auto& wheel_driver : ctx.wheel_drivers) {
                wheel_driver.initialize();
            }
        }

        inline void wheel_config_init() noexcept
        {
            constexpr auto WHEEL_DIST = 10.0F64;

            ctx.wheel_distance = WHEEL_DIST;
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
