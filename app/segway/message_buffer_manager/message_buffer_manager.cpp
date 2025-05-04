#include "message_buffer_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr auto MESSAGE_BUFFER_NUM =
            std::to_underlying(MessageBufferType::MESSAGE_BUFFER_NUM);

        auto message_buffers = std::array<MessageBufferHandle_t, MESSAGE_BUFFER_NUM>{};

    }; // namespace

    void set_message_buffer(MessageBufferType const type,
                            MessageBufferHandle_t const handle) noexcept
    {
        assert(handle);

        auto const index = std::to_underlying(type);
        assert(index < MESSAGE_BUFFER_NUM);
        assert(!message_buffers[index]);

        message_buffers[index] = handle;
    }

    MessageBufferHandle_t get_message_buffer(MessageBufferType const type) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < MESSAGE_BUFFER_NUM);
        assert(message_buffers[index]);

        return message_buffers[index];
    }

}; // namespace segway