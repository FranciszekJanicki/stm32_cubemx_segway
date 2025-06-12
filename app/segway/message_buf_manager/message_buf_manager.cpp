#include "message_buf_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr auto MESSAGE_BUFFER_NUM = std::to_underlying(MessageBufType::MESSAGE_BUFFER_NUM);

        auto message_bufs = std::array<MessageBufferHandle_t, MESSAGE_BUFFER_NUM>{};

    }; // namespace

    void set_message_buf(MessageBufType const type, MessageBufferHandle_t const handle) noexcept
    {
        assert(handle);

        auto const index = std::to_underlying(type);
        assert(index < MESSAGE_BUFFER_NUM);
        assert(!message_bufs[index]);

        message_bufs[index] = handle;
    }

    MessageBufferHandle_t get_message_buf(MessageBufType const type) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < MESSAGE_BUFFER_NUM);
        assert(message_bufs[index]);

        return message_bufs[index];
    }

}; // namespace segway