/**
 * @file rs485.h
 * @brief RS-485 transport (PIO UART + DE control)
 */

#ifndef RS485_H
#define RS485_H

#include "transport.h"
#include "hardware/pio.h"

/* ──────────────────────────────────────────────
 *  RS-485 transport data
 * ────────────────────────────────────────────── */
typedef struct {
    PIO pio;
    uint sm_tx;
    uint sm_rx;
    uint pin_tx;
    uint pin_rx;
    uint pin_de;              /* DE/RE# control pin (high = TX) */
} rs485_t;

/**
 * @brief Initialize RS-485 transport
 */
void rs485_init(transport_t *t, PIO pio, uint sm_tx, uint sm_rx,
                uint pin_tx, uint pin_rx, uint pin_de, uint32_t baud);

/**
 * @brief Get transport ops for RS-485
 */
const transport_ops_t *rs485_get_ops(void);

#endif /* RS485_H */