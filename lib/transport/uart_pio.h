/**
 * @file uart_pio.h
 * @brief PIO UART transport (full-duplex for DDSM210, RS-485, etc)
 */

#ifndef UART_PIO_H
#define UART_PIO_H

#include "transport.h"
#include "hardware/pio.h"

/* ──────────────────────────────────────────────
 *  PIO UART transport data
 * ────────────────────────────────────────────── */
typedef struct {
    PIO pio;
    uint sm_tx;
    uint sm_rx;
    uint pin_tx;
    uint pin_rx;
} uart_pio_t;

/**
 * @brief Initialize PIO UART transport
 */
void uart_pio_init(transport_t *t, PIO pio, uint sm_tx, uint sm_rx,
                   uint pin_tx, uint pin_rx, uint32_t baud);

/**
 * @brief Get transport ops for PIO UART
 */
const transport_ops_t *uart_pio_get_ops(void);

#endif /* UART_PIO_H */