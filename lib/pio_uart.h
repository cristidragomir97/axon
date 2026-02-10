/**
 * @file pio_uart.h
 * @brief PIO UART wrapper for easy initialization
 */

#ifndef PIO_UART_H
#define PIO_UART_H

#include "hardware/pio.h"
#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

/* Track program offsets per PIO instance */
static uint pio0_tx_offset = 0xFFFF;
static uint pio0_rx_offset = 0xFFFF;
static uint pio1_tx_offset = 0xFFFF;
static uint pio1_rx_offset = 0xFFFF;

/**
 * @brief Initialize a PIO UART (TX and RX state machines)
 *
 * Programs are only loaded once per PIO instance and shared across SMs.
 */
static inline void pio_uart_init(PIO pio, uint sm_tx, uint sm_rx,
                                  uint pin_tx, uint pin_rx, uint baud) {
    uint *tx_offset = (pio == pio0) ? &pio0_tx_offset : &pio1_tx_offset;
    uint *rx_offset = (pio == pio0) ? &pio0_rx_offset : &pio1_rx_offset;

    /* Load TX program if not already loaded */
    if (*tx_offset == 0xFFFF) {
        *tx_offset = pio_add_program(pio, &uart_tx_program);
    }

    /* Load RX program if not already loaded */
    if (*rx_offset == 0xFFFF) {
        *rx_offset = pio_add_program(pio, &uart_rx_program);
    }

    uart_tx_program_init(pio, sm_tx, *tx_offset, pin_tx, baud);
    uart_rx_program_init(pio, sm_rx, *rx_offset, pin_rx, baud);
}

/**
 * @brief Send a byte on PIO UART
 */
static inline void pio_uart_putc(PIO pio, uint sm_tx, char c) {
    uart_tx_program_putc(pio, sm_tx, c);
}

/**
 * @brief Send a buffer on PIO UART
 */
static inline void pio_uart_write(PIO pio, uint sm_tx, const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uart_tx_program_putc(pio, sm_tx, buf[i]);
    }
}

/**
 * @brief Check if RX data available
 */
static inline bool pio_uart_readable(PIO pio, uint sm_rx) {
    return uart_rx_program_available(pio, sm_rx);
}

/**
 * @brief Read a byte from PIO UART (blocking)
 */
static inline char pio_uart_getc(PIO pio, uint sm_rx) {
    return uart_rx_program_getc(pio, sm_rx);
}

/**
 * @brief Read a byte from PIO UART (non-blocking)
 * @return byte read, or -1 if no data available
 */
static inline int pio_uart_getc_nonblock(PIO pio, uint sm_rx) {
    if (pio_sm_is_rx_fifo_empty(pio, sm_rx)) {
        return -1;
    }
    return (int)(pio_sm_get(pio, sm_rx) >> 24);
}

#endif /* PIO_UART_H */