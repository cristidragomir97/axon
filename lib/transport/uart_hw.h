/**
 * @file uart_hw.h
 * @brief Hardware UART transport (half-duplex for Feetech/Dynamixel)
 */

#ifndef UART_HW_H
#define UART_HW_H

#include "transport.h"
#include "hardware/uart.h"

/* ──────────────────────────────────────────────
 *  Hardware UART transport data
 * ────────────────────────────────────────────── */
typedef struct {
    uart_inst_t *uart;
    uint pin_tx;
    uint pin_rx;
    /* Half-duplex: TX and RX are same wire via external circuit */
    bool half_duplex;
} uart_hw_t;

/**
 * @brief Initialize hardware UART transport
 */
void uart_hw_init(transport_t *t, uart_inst_t *uart,
                  uint pin_tx, uint pin_rx, uint32_t baud, bool half_duplex);

/**
 * @brief Get transport ops for hardware UART
 */
const transport_ops_t *uart_hw_get_ops(void);

#endif /* UART_HW_H */