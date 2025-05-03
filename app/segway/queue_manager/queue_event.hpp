#ifndef SEGWAY_QUEUE_EVENT_HPP
#define SEGWAY_QUEUE_EVENT_HPP

#include <array>
#include <cstdint>
#include <stdfloat>

namespace segway {

    enum struct ControlEventType : std::uint8_t {
        START,
        STOP,
        IMU_DATA,
    };

    union ControlEventPayload {
        struct {
        } start;

        struct {
        } stop;

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

    enum struct IMUEventType : std::uint8_t {
        START,
        STOP,
    };

    union IMUEventPayload {
        struct {
        } start;

        struct {
        } stop;
    };

    struct IMUEvent {
        IMUEventType type;
        IMUEventPayload payload;
    };

    enum struct WheelEventType : std::uint8_t {
        START,
        STOP,
        WHEEL_DATA,
    };

    union WheelEventPayload {
        struct {
        } start;

        struct {
        } stop;

        struct {
            std::float64_t left_speed;
            std::float64_t right_speed;
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