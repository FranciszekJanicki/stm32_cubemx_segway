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
        static char buffer[100];

        auto* buf = buffer;
        auto buf_len = sizeof(buffer);
        auto dynamic_buf = false;

        auto tag_len = std::strlen(tag) + std::strlen(": ");
        auto args_len = std::snprintf(nullptr, buf_len - tag_len, fmt, args...);
        auto endline_len = std::strlen("\n\r");
        auto len = tag_len + args_len + endline_len;

        // if (len > buf_len) {
        //     buf = static_cast<char*>(std::malloc(len));
        //     dynamic_buf = true;
        //     buf_len = len;
        // }

        if (buf) {
            std::memset(buf, '\0', buf_len);
            std::strncpy(buf, tag, buf_len);
            std::strncat(buf, ": ", buf_len);
            std::snprintf(buf + tag_len, buf_len - tag_len, fmt, args...);

            if (std::strlen(buf) < 2UL || std::strncmp(buf + std::strlen(buf) - std::strlen("\n\r"),
                                                       "\n\r",
                                                       std::strlen("\n\r")) != 0) {
                std::strncat(buf, "\n\r", buf_len);
            }

            xQueueSend(get_log_queue(), buf, portMAX_DELAY);

            // if (dynamic_buf) {
            //     std::free(buf);
            // }
        }
    }

}; // namespace segway

#endif // SEGWAY_LOG_HPP