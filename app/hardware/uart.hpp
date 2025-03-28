#ifndef UART_HPP
#define UART_HPP

#include "usart.h"

namespace Hardware {

    void initialize_uart2() noexcept;

    void deinitialize_uart2() noexcept;

}; // namespace Hardware

#endif // UART_HPP