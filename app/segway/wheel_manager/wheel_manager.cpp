#include "wheel_manager.hpp"
#include "event_group_manager.hpp"
#include "gpio.h"
#include "queue_manager.hpp"
#include "tim.h"
#include "utility.hpp"
#include "wheel.hpp"
#include <cassert>

namespace segway {

    namespace {

        constexpr auto TAG = "wheel_manager";

        using namespace stm32_utility;

        struct Context {
            std::array<Wheel, 2UL> wheels = {};

            struct Config {
                std::float64_t wheel_fault_thresh_low = {};
                std::float64_t wheel_fault_thresh_high = {};
                std::float64_t wheel_distance = {};
            } config;
        } ctx;

        WheelDriver& get_wheel_driver(WheelType const wheel_type) noexcept
        {
            auto* it = std::find_if(ctx.wheels.begin(), ctx.wheels.end(), [wheel_type](Wheel const& wheel) {
                return wheel.type == wheel_type;
            });

            assert(it != ctx.wheels.end());
            return it->driver;
        }

        void process_left_wheel_speed(WheelEventPayload const& payload) noexcept
        {
            auto& driver = get_wheel_driver(WheelType::LEFT);
            driver.set_wheel_speed(payload.left_wheel_data.speed, payload.left_wheel_data.dt);
        }

        void process_right_wheel_speed(WheelEventPayload const& payload) noexcept
        {
            auto& driver = get_wheel_driver(WheelType::RIGHT);
            driver.set_wheel_speed(payload.right_wheel_data.speed, payload.right_wheel_data.dt);
        }

        void process_queue_events() noexcept
        {
            auto event = WheelEvent{};
            auto queue = get_queue(QueueType::WHEEL);

            while (uxQueueMessagesWaiting(queue)) {
                if (xQueueReceive(queue, &event, pdMS_TO_TICKS(10))) {
                    switch (event.type) {
                        case WheelEventType::LEFT_WHEEL_SPEED:
                            process_left_wheel_speed(event.payload);
                            break;
                        case WheelEventType::RIGHT_WHEEL_SPEED:
                            process_right_wheel_speed(event.payload);
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        void process_left_pwm_pulse() noexcept
        {
            auto& driver = get_wheel_driver(WheelType::LEFT);
            driver.update_step_count();
        };

        void process_right_pwm_pulse() noexcept
        {
            auto& driver = get_wheel_driver(WheelType::RIGHT);
            driver.update_step_count();
        };

        void process_left_step_timer() noexcept
        {
            gpio_toggle_pin(GPIO::PA8);
        };

        void process_right_step_timer() noexcept
        {
            gpio_toggle_pin(GPIO::PA6);
        };

        void process_event_group_bits() noexcept
        {
            auto event_group = get_event_group(EventGroupType::WHEEL);

            auto event_bits = xEventGroupWaitBits(event_group, WheelEventBit::ALL, pdTRUE, pdFALSE, pdMS_TO_TICKS(10));

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
    }; // namespace

    void wheel_manager_init() noexcept
    {
        constexpr auto MS1_1 = std::to_underlying(GPIO::PA11);
        constexpr auto MS2_1 = std::to_underlying(GPIO::PA10);
        constexpr auto MS3_1 = std::to_underlying(GPIO::PA9);
        constexpr auto DIR_1 = std::to_underlying(GPIO::PB15);
        constexpr auto EN_1 = std::to_underlying(GPIO::NC);
        constexpr auto SLEEP_1 = std::to_underlying(GPIO::NC);
        constexpr auto RESET_1 = std::to_underlying(GPIO::NC);

        constexpr auto MS1_2 = std::to_underlying(GPIO::PA3);
        constexpr auto MS2_2 = std::to_underlying(GPIO::PA4);
        constexpr auto MS3_2 = std::to_underlying(GPIO::PA5);
        constexpr auto DIR_2 = std::to_underlying(GPIO::PA7);
        constexpr auto EN_2 = std::to_underlying(GPIO::NC);
        constexpr auto SLEEP_2 = std::to_underlying(GPIO::NC);
        constexpr auto RESET_2 = std::to_underlying(GPIO::NC);

        constexpr auto STEPS_PER_360 = 200U;
        constexpr auto WHEEL_DIST = 10.0F64;
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

        auto a4988_gpio_write_pin = [](void* user, std::int16_t pin, bool state) {
            gpio_write_pin(static_cast<GPIO>(pin), static_cast<GPIOState>(state));
        };

        auto a4988_pulse_start = [](void* user) {
            auto handle = static_cast<TIM_HandleTypeDef*>(user);
            HAL_TIM_Base_Start_IT(handle);
        };
        auto a4988_pulse_stop = [](void* user) {
            auto handle = static_cast<TIM_HandleTypeDef*>(user);
            HAL_TIM_Base_Stop_IT(handle);
        };
        auto a4988_pulse_set_freq = [](void* user, std::uint32_t freq) {
            if (freq > 0UL) {
                auto handle = static_cast<TIM_HandleTypeDef*>(user);
                auto prescaler = 0UL;
                auto period = 0UL;
                utility::frequency_to_prescaler_and_period(2 * freq, 84000000, 0, 0xFFFF, 0xFFFF, prescaler, period);
                handle->Instance->PSC = prescaler;
                handle->Instance->ARR = period;
            }
        };

        auto a4988_1_interface = a4988::Interface{.gpio_user = nullptr,
                                                  .gpio_write_pin = a4988_gpio_write_pin,
                                                  .pulse_user = &htim1,
                                                  .pulse_start = a4988_pulse_start,
                                                  .pulse_stop = a4988_pulse_stop,
                                                  .pulse_set_freq = a4988_pulse_set_freq};
        auto a4988_2_interface = a4988::Interface{.gpio_user = nullptr,
                                                  .gpio_write_pin = a4988_gpio_write_pin,
                                                  .pulse_user = &htim3,
                                                  .pulse_start = a4988_pulse_start,
                                                  .pulse_stop = a4988_pulse_stop,
                                                  .pulse_set_freq = a4988_pulse_set_freq};

        auto a4988_1 = a4988::A4988{.config = std::move(a4988_1_config), .interface = std::move(a4988_1_interface)};
        auto a4988_2 = a4988::A4988{.config = std::move(a4988_2_config), .interface = std::move(a4988_2_interface)};

        auto step_driver_1 = step_driver::StepDriver{.driver = std::move(a4988_1), .steps_per_360 = STEPS_PER_360};
        auto step_driver_2 = step_driver::StepDriver{.driver = std::move(a4988_2), .steps_per_360 = STEPS_PER_360};

        auto left_wheel = segway::Wheel{
            .type = segway::WheelType::LEFT,
            .driver = segway::WheelDriver{.driver = std::move(step_driver_1), .wheel_radius = WHEEL_RADIUS}};

        auto right_wheel = segway::Wheel{
            .type = segway::WheelType::RIGHT,
            .driver = segway::WheelDriver{.driver = std::move(step_driver_2), .wheel_radius = WHEEL_RADIUS}};

        ctx.wheels[0] = std::move(left_wheel);
        ctx.wheels[1] = std::move(right_wheel);

        for (auto& [type, driver] : ctx.wheels) {
            driver.initialize();
        }
    }

    void wheel_manager_process() noexcept
    {
        process_queue_events();
        process_event_group_bits();
    }
}; // namespace segway
