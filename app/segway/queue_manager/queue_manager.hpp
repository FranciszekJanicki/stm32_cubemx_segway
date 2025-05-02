#ifndef SEGWAY_QUEUE_MANAGER_HPP
#define SEGWAY_QUEUE_MANAGER_HPP

#include "FreeRTOS.h"
#include "queue.h"
#include "queue_event.hpp"
#include <cstdint>

namespace segway {

    enum struct QueueType : std::uint8_t {
        CONTROL,
        WHEEL,
        LOG,
        QUEUE_NUM,
    };

    void set_queue(QueueType const type, QueueHandle_t const handle) noexcept;

    inline void set_control_queue(QueueHandle_t const handle) noexcept
    {
        set_queue(QueueType::CONTROL, handle);
    }

    inline void set_wheel_queue(QueueHandle_t const handle) noexcept
    {
        set_queue(QueueType::WHEEL, handle);
    }

    inline void set_log_queue(QueueHandle_t const handle) noexcept
    {
        set_queue(QueueType::LOG, handle);
    }

    QueueHandle_t get_queue(QueueType const type) noexcept;

    inline QueueHandle_t get_control_queue() noexcept
    {
        return get_queue(QueueType::CONTROL);
    }

    inline QueueHandle_t get_wheel_queue() noexcept
    {
        return get_queue(QueueType::WHEEL);
    }

    inline QueueHandle_t get_log_queue() noexcept
    {
        return get_queue(QueueType::LOG);
    }

}; // namespace segway

#endif // SEGWAY_QUEUE_MANAGER_HPP