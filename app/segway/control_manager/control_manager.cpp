#include "control_manager.hpp"
#include "pid.hpp"
#include "queue_manager.hpp"
#include <array>
#include <variant>

namespace segway {

    namespace {

        constexpr auto TAG = "control_manager";

        struct LQR {
            std::array<std::float64_t, 6UL> Kx = {};
            std::array<std::float64_t, 6UL> Ki = {};

            std::array<std::float64_t, 6UL> prev_x = {};
            std::array<std::float64_t, 6UL> prev_e = {};
            std::array<std::float64_t, 6UL> int_e = {};
            std::array<std::float64_t, 6UL> x = {};
            std::array<std::float64_t, 6UL> e = {};
            std::array<std::float64_t, 2UL> u = {};
        };

        struct Context {
            struct Config {
                std::float64_t tilt_fault_thresh_low = {};
                std::float64_t tilt_fault_thresh_high = {};
            } config;

            utility::PID<std::float64_t> regulator = {};

            std::float64_t tilt = {};
            std::float64_t speed = {};
        } ctx;

        void process_imu_data(ControlEventPayload const& payload) noexcept
        {
            ctx.tilt = payload.imu_data.roll;

            ctx.speed = ctx.regulator(0.0F64 - ctx.tilt, payload.imu_data.dt);
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
    {}

    void control_manager_process() noexcept
    {
        process_queue_events();
    }

}; // namespace segway