#ifndef SEGWAY_WHEEL_QUEUE_HPP
#define SEGWAY_WHEEL_QUEUE_HPP

#include "queue_manager.hpp"
#include <cstdint>

namespace segway {

    constexpr auto WHEEL_QUEUE_ITEM_SIZE = sizeof(WheelEvent);
    constexpr auto WHEEL_QUEUE_ITEMS = 10UL;
    constexpr auto WHEEL_QUEUE_STORAGE_SIZE = WHEEL_QUEUE_ITEM_SIZE * WHEEL_QUEUE_ITEMS;

    void wheel_queue_init() noexcept;

}; // namespace segway

#endif // SEGWAY_WHEEL_QUEUE_HPP