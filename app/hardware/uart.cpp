#include "uart.hpp"

namespace Hardware {

    void initialize_uart2() noexcept
    {
        MX_USART2_UART_Init();
    }

    void deinitialize_uart2() noexcept
    {}

}; // namespace Hardware
