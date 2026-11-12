#include <stdint.h>

#include "uart.h"

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

int main(void);

#define SCB_CPACR (*(volatile uint32_t *)0xE000ED88u)

void Reset_Handler(void);
static void fault_handler(void);
static void default_handler(void);

// An entry of the vector table is either a handler or, for the first entry, the initial stack pointer
typedef union vector {
    void (*handler)(void);
    void *stack_top;
} vector_t;

// Cortex-M exceptions only. The test image uses no peripheral interrupts.
__attribute__((section(".isr_vector"), used)) static const vector_t vector_table[16] = {
    {.stack_top = &_estack}, // Initial stack pointer
    {.handler = Reset_Handler},
    {.handler = default_handler}, // NMI
    {.handler = fault_handler},   // HardFault
    {.handler = fault_handler},   // MemManage
    {.handler = fault_handler},   // BusFault
    {.handler = fault_handler},   // UsageFault
    {.handler = 0},
    {.handler = 0},
    {.handler = 0},
    {.handler = 0},
    {.handler = default_handler}, // SVCall
    {.handler = default_handler}, // DebugMon
    {.handler = 0},
    {.handler = default_handler}, // PendSV
    {.handler = default_handler}, // SysTick
};

void Reset_Handler(void)
{
    // Enable the FPU (CP10 and CP11 full access) before any floating point instruction
    SCB_CPACR |= (0xFu << 20);
    __asm volatile("dsb\n isb");

    for (uint32_t *src = &_sidata, *dst = &_sdata; dst < &_edata;) {
        *dst++ = *src++;
    }
    for (uint32_t *dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;
    }

    (void)main();

    while (1) {
    }
}

static void fault_handler(void)
{
    uart_puts("ULYSSES-SIM: FAIL cortex-m fault\n");
    while (1) {
    }
}

static void default_handler(void)
{
    while (1) {
    }
}
