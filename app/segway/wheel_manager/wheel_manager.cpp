#include "wheel_manager.hpp"
#include "event_group_manager.hpp"
#include "queue_manager.hpp"
#include "wheel.hpp"

namespace segway {

    namespace {

        constexpr auto TAG = "wheel_manager";

        struct Context {
            //  Wheels wheels = {};
        };

        auto ctx = Context{};

        void process_left_wheel_speed(WheelEventPayload const& payload) noexcept
        {}

        void process_right_wheel_speed(WheelEventPayload const& payload) noexcept
        {}

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

        void process_left_pwm_pulse() noexcept {};

        void process_right_pwm_pulse() noexcept {};

        void process_left_step_timer() noexcept {};

        void process_right_step_timer() noexcept {};

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
    {}

    void wheel_manager_process() noexcept
    {
        process_queue_events();
        process_event_group_bits();
    }

}; // namespace segway
