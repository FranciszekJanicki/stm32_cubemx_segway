#ifndef SEGWAY_MESSAGE_BUFFER_MANAGER_HPP
#define SEGWAY_MESSAGE_BUFFER_MANAGER_HPP

#include "FreeRTOS.h"
#include "message_buffer.h"
#include <cstdint>

namespace segway {

    enum struct MessageBufferType : std::uint8_t {
        CONTROL,
        IMU,
        WHEEL,
        MESSAGE_BUFFER_NUM,
    };

    void set_message_buffer(MessageBufferType const type,
                            MessageBufferHandle_t const handle) noexcept;

    MessageBufferHandle_t get_message_buffer(MessageBufferType const type) noexcept;

}; // namespace segway

#endif // SEGWAY_MESSAGE_BUFFER_MANAGER_HPP