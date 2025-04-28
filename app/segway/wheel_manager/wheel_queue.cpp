#include "wheel_queue.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "queue_manager.hpp"
#include <array>

namespace segway {

    namespace {

        inline auto wheel_static_queue = StaticQueue_t{};
        inline auto wheel_queue_storage = std::array<std::uint8_t, WHEEL_QUEUE_STORAGE_SIZE>{};

    }; // namespace

    void wheel_queue_init() noexcept
    {
        auto handle = xQueueCreateStatic(WHEEL_QUEUE_ITEMS,
                                         WHEEL_QUEUE_ITEM_SIZE,
                                         wheel_queue_storage.data(),
                                         &wheel_static_queue);

        set_queue(QueueType::WHEEL, handle);
    }

}; // namespace segway