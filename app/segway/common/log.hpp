#ifndef SEGWAY_LOG_HPP
#define SEGWAY_LOG_HPP

#include "queue_manager.hpp"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace segway {

    inline void LOG(char const* tag, char const* fmt, auto const... args) noexcept
    {
        auto event = LogEvent{};

        auto* buf = event.buffer;
        auto buf_len = sizeof(event.buffer);
        auto alloc_buf = false;

        auto tag_len = std::strlen(tag) + std::strlen(": ");
        auto args_len = std::snprintf(nullptr, 0, fmt, args...);
        auto endline = std::strstr(fmt, "\n\r") ? "" : "\n\r";
        auto endline_len = std::strlen(endline);
        auto len = tag_len + args_len + endline_len + 1;

        if (len > buf_len) {
            buf = static_cast<char*>(std::calloc(len, 1));
            buf_len = len;
            alloc_buf = true;
        }

        if (buf) {
            std::memset(buf, 0, buf_len);
            std::strncpy(buf, tag, buf_len);
            std::strncat(buf, ": ", buf_len - tag_len);
            std::snprintf(buf + tag_len, buf_len - tag_len, fmt, args...);
            std::strncat(buf, endline, buf_len - std::strlen(buf));

            xQueueSend(get_log_queue(), &event, portMAX_DELAY);

            if (alloc_buf) {
                std::free(buf);
            }
        }
    }

}; // namespace segway

#endif // SEGWAY_LOG_HPP