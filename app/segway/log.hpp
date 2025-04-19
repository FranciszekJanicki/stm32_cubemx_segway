#ifndef LOG_HPP
#define LOG_HPP

#include "log.hpp"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace Segway {

    template <typename... Args>
    inline void LOG(char const* tag, char const* fmt, Args const... args) noexcept
    {
        static char buffer[100];

        auto* buf = buffer;
        auto buf_len = sizeof(buffer);
        auto dynamic_buf = false;

        auto prefix_len = std::strlen(tag) + std::strlen(": ");
        auto len = prefix_len + std::snprintf(nullptr, buf_len - prefix_len, fmt, args...);

        if (len > buf_len) {
            buf = static_cast<char*>(std::malloc(len));
            dynamic_buf = true;
            buf_len = len;
        }

        if (buf) {
            std::memset(buf, '\0', buf_len);
            std::strncpy(buf, tag, buf_len);
            std::strncat(buf, ": ", buf_len);
            std::snprintf(buf + prefix_len, buf_len - prefix_len, fmt, args...);

            HAL_UART_Transmit(&huart2, reinterpret_cast<std::uint8_t*>(buf), strlen(buf), strlen(buf));
            CDC_Transmit_FS(reinterpret_cast<std::uint8_t*>(buf), strlen(buf));

            if (dynamic_buf) {
                std::free(buf);
            }
        }
    }

}; // namespace Segway

#endif // LOG_HPP