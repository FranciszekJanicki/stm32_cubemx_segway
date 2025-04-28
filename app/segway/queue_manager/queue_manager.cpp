#include "queue_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr auto QUEUE_NUM = std::to_underlying(QueueType::QUEUE_NUM);

        auto queue_handles = std::array<QueueHandle_t, QUEUE_NUM>{};

    }; // namespace

    void set_queue_handle(QueueType const type, QueueHandle_t const handle) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < QUEUE_NUM);
        assert(queue_handles[index] == nullptr);

        queue_handles[index] = handle;
    }

    QueueHandle_t get_queue_handle(QueueType const type) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < QUEUE_NUM);
        assert(queue_handles[index] != nullptr);

        return queue_handles[index];
    }

}; // namespace segway