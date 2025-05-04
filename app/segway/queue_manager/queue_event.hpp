#ifndef SEGWAY_QUEUE_EVENT_HPP
#define SEGWAY_QUEUE_EVENT_HPP

#include <array>
#include <cstdint>
#include <stdfloat>

namespace segway {

    enum struct ControlEventType : std::uint8_t {
        IMU_DATA,
        WHEEL_DATA,
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

    enum struct IMUEventType : std::uint8_t {};

    union IMUEventPayload {};

    struct IMUEvent {
        IMUEventType type;
        IMUEventPayload payload;
    };

    enum struct WheelEventType : std::uint8_t {
        CONTROL_DATA,
    };

    union WheelEventPayload {
        struct {
            std::float64_t left_speed;
            std::float64_t right_speed;
            std::float64_t dt;

            bool should_run;
        } control_data;
    };

    struct WheelEvent {
        WheelEventType type;
        WheelEventPayload payload;
    };

    struct LogEvent {
        char buf[100UL];
    };

}; // namespace segway

#endif // SEGWAY_QUEUE_EVENT_HPP