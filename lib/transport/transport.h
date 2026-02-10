/**
 * @file transport.h
 * @brief Transport layer abstraction for motor communication
 *
 * Transports handle the physical layer (UART, RS-485, etc).
 * Protocols use transports to send/receive packets.
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ──────────────────────────────────────────────
 *  Transport types
 * ────────────────────────────────────────────── */
typedef enum {
    TRANSPORT_UART_HW,        /* Hardware UART (half-duplex) */
    TRANSPORT_UART_PIO,       /* PIO UART (full-duplex) */
    TRANSPORT_RS485,          /* RS-485 with DE control */
} transport_type_t;

/* ──────────────────────────────────────────────
 *  Transport interface
 * ────────────────────────────────────────────── */
struct transport_s;
typedef struct transport_s transport_t;

typedef struct {
    /* Write data to transport */
    bool (*write)(transport_t *t, const uint8_t *data, size_t len);

    /* Read available data (non-blocking) */
    size_t (*read)(transport_t *t, uint8_t *buf, size_t max_len);

    /* Check if data available */
    bool (*readable)(transport_t *t);

    /* Flush TX buffer */
    void (*flush)(transport_t *t);

    /* Set direction for half-duplex (RS-485) */
    void (*set_tx_mode)(transport_t *t, bool tx);
} transport_ops_t;

struct transport_s {
    transport_type_t type;
    const transport_ops_t *ops;
    void *hw;                 /* Hardware handle (UART, PIO, etc) */
    uint32_t baud;
};

/* ──────────────────────────────────────────────
 *  Transport instances (defined in generated config)
 * ────────────────────────────────────────────── */
extern transport_t transport_feetech;
extern transport_t transport_dynamixel;
extern transport_t transport_ddsm210;
extern transport_t transport_rs485;

/* ──────────────────────────────────────────────
 *  Convenience macros
 * ────────────────────────────────────────────── */
static inline bool transport_write(transport_t *t, const uint8_t *data, size_t len) {
    return t->ops->write(t, data, len);
}

static inline size_t transport_read(transport_t *t, uint8_t *buf, size_t max_len) {
    return t->ops->read(t, buf, max_len);
}

static inline bool transport_readable(transport_t *t) {
    return t->ops->readable(t);
}

static inline void transport_flush(transport_t *t) {
    if (t->ops->flush) t->ops->flush(t);
}

static inline void transport_set_tx_mode(transport_t *t, bool tx) {
    if (t->ops->set_tx_mode) t->ops->set_tx_mode(t, tx);
}

#endif /* TRANSPORT_H */