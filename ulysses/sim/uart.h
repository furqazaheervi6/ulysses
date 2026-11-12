#ifndef ULYSSES_SIM_UART
#define ULYSSES_SIM_UART

/** @brief Sets up the console UART. Call once before uart_puts(). */
void uart_init(void);

/** @brief Writes a NUL-terminated string to the console UART, blocking */
void uart_puts(const char *s);

#endif /* ULYSSES_SIM_UART */
