#include "queue_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr std::size_t QUEUE_NUM = std::to_underlying(QueueType::QUEUE_NUM);

        std::array<QueueHandle_t, QUEUE_NUM> queues = {};

    }; // namespace

    void set_queue(QueueType const type, QueueHandle_t const handle) noexcept
    {
        assert(handle);

        auto index = std::to_underlying(type);
        assert(index < QUEUE_NUM);
        assert(!queues[index]);

        queues[index] = handle;
    }

    QueueHandle_t get_queue(QueueType const type) noexcept
    {
        auto index = std::to_underlying(type);
        assert(index < QUEUE_NUM);
        assert(queues[index]);

        return queues[index];
    }

}; // namespace segway