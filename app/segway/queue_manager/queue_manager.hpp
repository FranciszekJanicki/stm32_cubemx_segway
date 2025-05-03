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

    QueueHandle_t get_queue(QueueType const type) noexcept;

}; // namespace segway

#endif // SEGWAY_QUEUE_MANAGER_HPP