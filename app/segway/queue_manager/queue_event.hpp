#ifndef SEGWAY_QUEUE_EVENT_HPP
#define SEGWAY_QUEUE_EVENT_HPP

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
        LEFT_WHEEL_SPEED,
        RIGHT_WHEEL_SPEED,
        NONE,
    };

    union WheelEventPayload {
        struct {
            std::float64_t speed;
            std::float64_t dt;
        } left_wheel_data, right_wheel_data;
    };

    struct WheelEvent {
        WheelEventType type;
        WheelEventPayload payload;
    };

}; // namespace segway

#endif // SEGWAY_QUEUE_EVENT_HPP