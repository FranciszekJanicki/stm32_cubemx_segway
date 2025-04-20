#ifndef LOG_HPP
#define LOG_HPP

#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

    constexpr auto DEBUG_ENV = true;

};

namespace Segway {

    template <typename... Args>
    inline void LOG(char const* tag, char const* fmt, Args const... args) noexcept
    {
        if constexpr (DEBUG_ENV) {
            static char buffer[100];

            auto* buf = buffer;
            auto buf_len = sizeof(buffer);
            auto dynamic_buf = false;

            auto tag_len = std::strlen(tag) + std::strlen(": ");
            auto args_len = std::snprintf(nullptr, buf_len - tag_len, fmt, args...);
            auto endline_len = std::strlen("\n\r");

            auto len = tag_len + args_len + endline_len;

            if (len > buf_len) {
                buf = static_cast<char*>(std::malloc(len));
                dynamic_buf = true;
                buf_len = len;
            }

            if (buf) {
                std::memset(buf, '\0', buf_len);
                std::strncpy(buf, tag, buf_len);
                std::strncat(buf, ": ", buf_len);
                std::snprintf(buf + tag_len, buf_len - tag_len, fmt, args...);
                // append return carriage and endline characters if they are not present alredy

                auto msg_len = std::strlen(buf);
                if (msg_len < 2UL ||
                    std::strncmp(buf + msg_len - std::strlen("\n\r"), "\n\r", std::strlen("\n\r")) != 0) {
                    std::strncat(buf, "\n\r", buf_len);
                }

                HAL_UART_Transmit(&huart2, reinterpret_cast<std::uint8_t*>(buf), std::strlen(buf), std::strlen(buf));
                CDC_Transmit_FS(reinterpret_cast<std::uint8_t*>(buf), std::strlen(buf));

                if (dynamic_buf) {
                    std::free(buf);
                }
            }
        }
    }

}; // namespace Segway

#endif // LOG_HPP