#ifndef SEGWAY_QUEUE_EVENT_HPP
#define SEGWAY_QUEUE_EVENT_HPP

#include <array>
#include <cstdint>
#include <stdfloat>

namespace segway {

    enum struct ControlEventType : std::uint8_t {
        IMU_DATA,
        NONE,
    };

    union ControlEventPayload {
        struct {
            std::float64_t roll;
            std::float64_t pitch;
            std::float64_t yaw;
            std::float64_t dt;
        } imu_data;
    };

    struct ControlEvent {
        ControlEventType type;
        ControlEventPayload payload;
    };

    // 1:
    enum struct WheelEventType : std::uint8_t {
        WHEEL_DATA,
        NONE,
    };

    union WheelEventPayload {
        struct {
            std::float64_t left_wheel_speed;
            std::float64_t right_wheel_speed;
            std::float64_t dt;
        } wheel_data;
    };

    struct WheelEvent {
        WheelEventType type;
        WheelEventPayload payload;
    };

    struct LogEvent {
        char buffer[100UL];
    };

}; // namespace segway

#endif // SEGWAY_QUEUE_EVENT_HPP