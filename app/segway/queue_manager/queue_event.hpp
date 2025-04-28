#ifndef SEGWAY_QUEUE_EVENT_HPP
#define SEGWAY_QUEUE_EVENT_HPP

#include <cstdint>
#include <stdfloat>

namespace segway {

    struct ControlQueueEvent {
        enum Type : std::uint8_t {
            IMU_DATA,
        } type;

        union Payload {
            struct {
                std::float64_t roll;
                std::float64_t pitch;
                std::float64_t yaw;
            } imu_data;
        } payload;
    };

    struct WheelQueueEvent {
        enum Type : std::uint8_t {
            LEFT_WHEEL_SPEED,
            RIGHT_WHEEL_SPEED,
        } type;

        union Payload {
            std::float64_t left_wheel_speed;
            std::float64_t right_wheel_speed;
        } payload;
    };

}; // namespace segway

#endif // SEGWAY_QUEUE_EVENT_HPP