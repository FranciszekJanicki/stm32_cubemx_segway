#include "control_manager.hpp"
#include "queue_manager.hpp"

namespace segway {

    namespace {

        constexpr auto TAG = "control_manager";

        struct Context {
            struct Config {
                std::float64_t tilt_fault_thresh_low = {};
                std::float64_t tilt_fault_thresh_high = {};
            } config;
        } ctx;

        void process_imu_data(ControlEventPayload const& payload) noexcept
        {}

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