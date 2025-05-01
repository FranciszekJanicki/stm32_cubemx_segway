#ifndef SEGWAY_LOG_HPP
#define SEGWAY_LOG_HPP

#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define DEBUG_ENV

namespace {

#ifdef DEBUG_ENV
    inline auto static_mutex = StaticSemaphore_t{};
    inline auto mutex = SemaphoreHandle_t{};
#endif

}; // namespace

namespace segway {

    inline void log_init() noexcept
    {
#ifdef DEBUG_ENV
        mutex = xSemaphoreCreateMutexStatic(&static_mutex);
#endif // DEBUG_ENV
    }

    inline void LOG(char const* tag, char const* fmt, auto const... args) noexcept
    {
#ifdef DEBUG_ENV
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10))) {
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

                if (std::strlen(buf) < 2UL ||
                    std::strncmp(buf + std::strlen(buf) - std::strlen("\n\r"),
                                 "\n\r",
                                 std::strlen("\n\r")) != 0) {
                    std::strncat(buf, "\n\r", buf_len);
                }

                auto msg_len = std::strlen(buf);
                auto msg = reinterpret_cast<std::uint8_t*>(buf);

                HAL_UART_Transmit(&huart2, msg, msg_len, 100);
                CDC_Transmit_FS(msg, msg_len);

                if (dynamic_buf) {
                    std::free(buf);
                }
            }

            xSemaphoreGive(mutex);
        }
#endif // DEBUG_ENV
    }
}; // namespace segway

#endif // SEGWAY_LOG_HPP