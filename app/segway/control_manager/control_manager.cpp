#include "control_manager.hpp"
#include "pid.hpp"
#include "queue_manager.hpp"
#include <array>
#include <variant>

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
            auto tilt = payload.imu_data.roll;
            auto error_tilt = PID_Y_REF - tilt;
            auto control_speed = ctx.regulator(error_tilt, payload.imu_data.dt);

            auto queue = get_queue(QueueType::CONTROL);
            auto event = WheelEvent{.type = WheelEventType::WHEEL_DATA};
            event.payload.wheel_data = {.left_wheel_speed = control_speed,
                                        .right_wheel_speed = -control_speed,
                                        .dt = payload.imu_data.dt};

            xQueueSend(queue, &event, pdMS_TO_TICKS(10));
        }

        void process_queue_events() noexcept
        {
            auto event = ControlEvent{};
            auto queue = get_queue(QueueType::CONTROL);

            while (uxQueueMessagesWaiting(queue)) {
                if (xQueueReceive(queue, &event, pdMS_TO_TICKS(10))) {
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

    }; // namespace

    void control_manager_init() noexcept
    {
        ctx.regulator.kP = PID_KP;
        ctx.regulator.kI = PID_KI;
        ctx.regulator.kD = PID_KD;
        ctx.regulator.kC = PID_KC;
        ctx.regulator.sat = PID_SAT;
    }

    void control_manager_process() noexcept
    {
        process_queue_events();
    }
}; // namespace segway