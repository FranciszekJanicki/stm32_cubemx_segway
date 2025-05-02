#include "wheel_manager.hpp"
#include "FreeRTOS.h"
#include "event_group_manager.hpp"
#include "event_groups.h"
#include "gpio.h"
#include "log.hpp"
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

        struct Context {
            std::array<Wheel, 2UL> wheels;

            struct Config {
                std::float64_t fault_thresh_low;
                std::float64_t fault_thresh_high;
                std::float64_t wheel_distance;
            } config;
        } ctx;

        WheelDriver& get_wheel_driver(WheelType const type) noexcept
        {
            auto* it = std::find_if(ctx.wheels.begin(),
                                    ctx.wheels.end(),
                                    [type](Wheel const& wheel) { return wheel.type == type; });

            assert(it != ctx.wheels.end());
            return it->driver;
        }

        void start_left_pwm_timer() noexcept
        {
            HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
        }

        void stop_left_pwm_timer() noexcept
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
        }

        void start_right_pwm_timer() noexcept
        {
            HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
        }

        void stop_right_pwm_timer() noexcept
        {
            HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
        }

        void start_left_step_timer() noexcept
        {
            HAL_TIM_Base_Start_IT(&htim1);
        }

        void stop_left_step_timer() noexcept
        {
            HAL_TIM_Base_Stop_IT(&htim1);
        }

        void start_right_step_timer() noexcept
        {
            HAL_TIM_Base_Stop_IT(&htim3);
        }

        void stop_right_step_timer() noexcept
        {
            HAL_TIM_Base_Stop_IT(&htim3);
        }

        void process_left_wheel_data(WheelEventPayload const& payload) noexcept
        {
            auto& driver = get_wheel_driver(WheelType::LEFT);
            driver.set_wheel_speed(payload.wheel_data.left_speed, payload.wheel_data.dt);
        }

        void process_right_wheel_data(WheelEventPayload const& payload) noexcept
        {
            auto& driver = get_wheel_driver(WheelType::RIGHT);
            driver.set_wheel_speed(payload.wheel_data.right_speed, payload.wheel_data.dt);
        }

        void process_wheel_data(WheelEventPayload const& payload) noexcept
        {
            LOG(TAG, "process_wheel_data");

            process_left_wheel_data(payload);
            process_right_wheel_data(payload);
        }

        void process_wheel_queue_events() noexcept
        {
            LOG(TAG, "process_wheel_queue_events");

            auto event = WheelEvent{};

            while (uxQueueMessagesWaiting(get_wheel_queue())) {
                if (xQueueReceive(get_wheel_queue(), &event, pdMS_TO_TICKS(10))) {
                    switch (event.type) {
                        case WheelEventType::WHEEL_DATA:
                            process_wheel_data(event.payload);
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        void process_left_pwm_pulse() noexcept
        {
            LOG(TAG, "process_left_pwm_pulse");

            auto& driver = get_wheel_driver(WheelType::LEFT);
            driver.update_step_count();

            start_left_pwm_timer();
        };

        void process_right_pwm_pulse() noexcept
        {
            LOG(TAG, "process_right_pwm_pulse");

            auto& driver = get_wheel_driver(WheelType::RIGHT);
            driver.update_step_count();

            start_right_pwm_timer();
        };

        void process_left_step_timer() noexcept
        {
            LOG(TAG, "process_left_step_timer");

            HAL_GPIO_TogglePin(GPIOA, 1 << 8);

            start_left_step_timer();
        };

        void process_right_step_timer() noexcept
        {
            LOG(TAG, "process_right_step_timer");

            HAL_GPIO_TogglePin(GPIOA, 1 << 6);
            start_right_step_timer();

            start_right_step_timer();
        };

        void process_wheel_event_group_bits() noexcept
        {
            LOG(TAG, "process_event_group_bits");

            auto event_bits = xEventGroupWaitBits(get_wheel_event_group(),
                                                  WheelEventBit::ALL,
                                                  pdTRUE,
                                                  pdFALSE,
                                                  pdMS_TO_TICKS(10));

            if ((event_bits & WheelEventBit::LEFT_PWM_PULSE) == WheelEventBit::LEFT_PWM_PULSE) {
                process_left_pwm_pulse();
            }

            if ((event_bits & WheelEventBit::RIGHT_PWM_PULSE) == WheelEventBit::RIGHT_PWM_PULSE) {
                process_right_pwm_pulse();
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
            constexpr auto WHEEL_TASK_PRIORITY = 1UL;
            constexpr auto WHEEL_TASK_STACK_DEPTH = 1024UL;
            constexpr auto WHEEL_TASK_NAME = "wheel_task";
            constexpr auto WHEEL_TASK_ARG = nullptr;

            static auto wheel_static_task = StaticTask_t{};
            static auto wheel_task_stack = std::array<StackType_t, WHEEL_TASK_STACK_DEPTH>{};

            set_wheel_task(xTaskCreateStatic(&wheel_task,
                                             WHEEL_TASK_NAME,
                                             wheel_task_stack.size(),
                                             WHEEL_TASK_ARG,
                                             WHEEL_TASK_PRIORITY,
                                             wheel_task_stack.data(),
                                             &wheel_static_task));
        }

        inline void wheel_queue_init() noexcept
        {
            constexpr auto WHEEL_QUEUE_ITEM_SIZE = sizeof(WheelEvent);
            constexpr auto WHEEL_QUEUE_ITEMS = 10UL;
            constexpr auto WHEEL_QUEUE_STORAGE_SIZE = WHEEL_QUEUE_ITEM_SIZE * WHEEL_QUEUE_ITEMS;

            static auto wheel_static_queue = StaticQueue_t{};
            static auto wheel_queue_storage = std::array<std::uint8_t, WHEEL_QUEUE_STORAGE_SIZE>{};

            set_wheel_queue(xQueueCreateStatic(WHEEL_QUEUE_ITEMS,
                                               WHEEL_QUEUE_ITEM_SIZE,
                                               wheel_queue_storage.data(),
                                               &wheel_static_queue));
        }

        inline void wheel_event_group_init() noexcept
        {
            static auto wheel_static_event_group = StaticEventGroup_t{};

            set_wheel_event_group(xEventGroupCreateStatic(&wheel_static_event_group));
        }

    }; // namespace

    void wheel_manager_init() noexcept
    {
        LOG(TAG, "manager_init");

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
        constexpr auto WHEEL_DIST = 10.0F64;
        constexpr auto WHEEL_RADIUS = 1.0F64;

        constexpr auto FAULT_THRESH_HIGH = 1000.0F64;
        constexpr auto FAULT_THRESH_LOW = 800.0F64;

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
                utility::frequency_to_prescaler_and_period(2 * freq,
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

        auto step_driver_1 =
            step_driver::StepDriver{.driver = std::move(a4988_1), .steps_per_360 = STEPS_PER_360};
        auto step_driver_2 =
            step_driver::StepDriver{.driver = std::move(a4988_2), .steps_per_360 = STEPS_PER_360};

        auto left_wheel =
            segway::Wheel{.type = segway::WheelType::LEFT,
                          .driver = segway::WheelDriver{.driver = std::move(step_driver_1),
                                                        .wheel_radius = WHEEL_RADIUS}};

        auto right_wheel =
            segway::Wheel{.type = segway::WheelType::RIGHT,
                          .driver = segway::WheelDriver{.driver = std::move(step_driver_2),
                                                        .wheel_radius = WHEEL_RADIUS}};

        ctx.wheels[0] = std::move(left_wheel);
        ctx.wheels[1] = std::move(right_wheel);

        for (auto& [type, driver] : ctx.wheels) {
            driver.initialize();
        }

        wheel_queue_init();
        wheel_event_group_init();
        wheel_task_init();
    }

}; // namespace segway
