#ifndef SEGWAY_MESSAGE_BUF_MANAGER_HPP
#define SEGWAY_MESSAGE_BUF_MANAGER_HPP

#include "FreeRTOS.h"
#include "message_buffer.h"
#include <cstdint>

namespace segway {

    enum struct MessageBufType : std::uint8_t {
        CONTROL,
        WHEEL,
        MESSAGE_BUFFER_NUM,
    };

    void set_message_buf(MessageBufType const type, MessageBufferHandle_t const handle) noexcept;

    MessageBufferHandle_t get_message_buf(MessageBufType const type) noexcept;

}; // namespace segway

#endif // SEGWAY_MESSAGE_BUF_MANAGER_HPP