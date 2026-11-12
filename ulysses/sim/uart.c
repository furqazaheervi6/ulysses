#include <stdint.h>

#include "uart.h"

// Console UART: USART3, register level so the image does not depend on the HAL. Change SIM_UART_BASE
// if the simulator exposes the console on another USART. Not verified against Simantic yet.
#ifndef SIM_UART_BASE
#define SIM_UART_BASE 0x40004800u // USART3
#endif
#define SIM_UART_RCC_EN_BIT (1u << 18) // RCC_APB1LENR.USART3EN

#define RCC_APB1LENR (*(volatile uint32_t *)0x580244E8u)

#define UART_CR1 (*(volatile uint32_t *)(SIM_UART_BASE + 0x00u))
#define UART_BRR (*(volatile uint32_t *)(SIM_UART_BASE + 0x0Cu))
#define UART_ISR (*(volatile uint32_t *)(SIM_UART_BASE + 0x1Cu))
#define UART_TDR (*(volatile uint32_t *)(SIM_UART_BASE + 0x28u))

#define UART_CR1_UE (1u << 0)
#define UART_CR1_TE (1u << 3)
#define UART_ISR_TXE (1u << 7)

// Stops a lost TXE flag from hanging the image forever
#define TXE_SPIN_LIMIT 100000u

void uart_init(void)
{
    RCC_APB1LENR |= SIM_UART_RCC_EN_BIT;
    (void)RCC_APB1LENR; // Read back so the clock is on before the first access

    UART_BRR = 556u; // 64 MHz HSI / 115200
    UART_CR1 = UART_CR1_UE | UART_CR1_TE;
}

void uart_puts(const char *s)
{
    while (*s != '\0') {
        uint32_t spin = TXE_SPIN_LIMIT;
        while (((UART_ISR & UART_ISR_TXE) == 0u) && (spin > 0u)) {
            spin--;
        }
        UART_TDR = (uint32_t)(unsigned char)*s++;
    }
}
