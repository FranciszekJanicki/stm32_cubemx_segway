#include "queue_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr auto QUEUE_NUM = std::to_underlying(QueueType::QUEUE_NUM);

        auto queues = std::array<QueueHandle_t, QUEUE_NUM>{};

    }; // namespace

    void set_queue(QueueType const type, QueueHandle_t const handle) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < QUEUE_NUM);
        assert(queues[index] == nullptr);

        queues[index] = handle;
    }

    QueueHandle_t get_queue(QueueType const type) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < QUEUE_NUM);
        assert(queues[index] != nullptr);

        return queues[index];
    }

}; // namespace segway