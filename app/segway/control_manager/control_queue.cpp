#include "control_queue.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "queue_manager.hpp"
#include <array>

namespace segway {

    namespace {

        inline auto control_static_queue = StaticQueue_t{};
        inline auto control_queue_storage = std::array<std::uint8_t, CONTROL_QUEUE_STORAGE_SIZE>{};

    }; // namespace

    void control_queue_init() noexcept
    {
        auto handle = xQueueCreateStatic(CONTROL_QUEUE_ITEMS,
                                         CONTROL_QUEUE_ITEM_SIZE,
                                         control_queue_storage.data(),
                                         &control_static_queue);

        set_queue_handle(QueueType::CONTROL_QUEUE, handle);
    }

}; // namespace segway