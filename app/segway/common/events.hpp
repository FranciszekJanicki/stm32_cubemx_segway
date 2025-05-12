#ifndef SEGWAY_EVENTS_HPP
#define SEGWAY_EVENTS_HPP

#include "FreeRTOS.h"
#include "semphr.h"
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <stdfloat>

#define USE_QUEUES

namespace segway {

    enum struct ControlEventType : std::uint8_t {
        IMU_DATA,
        CONTROL_DATA,
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

    /* implementing experimental shit */

    namespace {

        ControlEvent control_event = {};
        WheelEvent wheel_event = {};

        SemaphoreHandle_t control_mutex = {};
        SemaphoreHandle_t wheel_mutex = {};

    }; // namespace

    inline void mutexes_init() noexcept
    {
        std::memset(&control_event, 0, sizeof(control_event));
        std::memset(&wheel_event, 0, sizeof(wheel_event));

        static StaticSemaphore_t control_mutex_buffer;
        static StaticSemaphore_t wheel_mutex_buffer;

        xSemaphoreCreateMutexStatic(&wheel_mutex_buffer);
        xSemaphoreCreateMutexStatic(&control_mutex_buffer);
    }

    inline void set_control_event(ControlEvent const* const event) noexcept
    {
        assert(event);

        if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(1))) {
            std::memcpy(&control_event, event, sizeof(control_event));
            xSemaphoreGive(control_mutex);
        }
    }

    inline void set_wheel_event(WheelEvent const* const event) noexcept
    {
        assert(event);

        if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(1))) {
            std::memcpy(&control_event, event, sizeof(control_event));
            xSemaphoreGive(control_mutex);
        }
    }

    inline void get_control_event(ControlEvent* const event) noexcept
    {
        assert(event);

        if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(1))) {
            std::memcpy(event, &control_event, sizeof(event));
            xSemaphoreGive(control_mutex);
        }
    }

    inline void get_wheel_event(WheelEvent* const event) noexcept
    {
        assert(event);

        if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(1))) {
            std::memcpy(event, &control_event, sizeof(event));
            xSemaphoreGive(control_mutex);
        }
    }

} // namespace segway

#endif // SEGWAY_EVENTS_HPP