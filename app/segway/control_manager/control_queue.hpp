#ifndef SEGWAY_CONTROL_QUEUE_HPP
#define SEGWAY_CONTROL_QUEUE_HPP

#include "queue_manager.hpp"
#include <cstdint>

namespace segway {

    constexpr auto CONTROL_QUEUE_ITEM_SIZE = sizeof(ControlQueueEvent);
    constexpr auto CONTROL_QUEUE_ITEMS = 10UL;
    constexpr auto CONTROL_QUEUE_STORAGE_SIZE = CONTROL_QUEUE_ITEM_SIZE * CONTROL_QUEUE_ITEMS;

    void control_queue_init() noexcept;

}; // namespace segway

#endif // SEGWAY_CONTROL_QUEUE_HPP