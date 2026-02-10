/**
 * @file uart_pio.c
 * @brief PIO UART transport implementation
 */

#include "uart_pio.h"
#include "pico/stdlib.h"
#include "pio_uart.h"

/* ──────────────────────────────────────────────
 *  Transport operations
 * ────────────────────────────────────────────── */
static bool uart_pio_write(transport_t *t, const uint8_t *data, size_t len) {
    uart_pio_t *pio = (uart_pio_t *)t->hw;

    for (size_t i = 0; i < len; i++) {
        pio_uart_putc(pio->pio, pio->sm_tx, data[i]);
    }

    return true;
}

static size_t uart_pio_read(transport_t *t, uint8_t *buf, size_t max_len) {
    uart_pio_t *pio = (uart_pio_t *)t->hw;
    size_t count = 0;

    while (count < max_len && pio_uart_readable(pio->pio, pio->sm_rx)) {
        int c = pio_uart_getc_nonblock(pio->pio, pio->sm_rx);
        if (c >= 0) {
            buf[count++] = (uint8_t)c;
        }
    }

    return count;
}

static bool uart_pio_readable(transport_t *t) {
    uart_pio_t *pio = (uart_pio_t *)t->hw;
    return pio_uart_readable(pio->pio, pio->sm_rx);
}

static void uart_pio_flush(transport_t *t) {
    /* PIO UART doesn't have a blocking flush, TX FIFO drains automatically */
    (void)t;
}

static const transport_ops_t uart_pio_ops = {
    .write = uart_pio_write,
    .read = uart_pio_read,
    .readable = uart_pio_readable,
    .flush = uart_pio_flush,
    .set_tx_mode = NULL,
};

/* ──────────────────────────────────────────────
 *  Public API
 * ────────────────────────────────────────────── */
const transport_ops_t *uart_pio_get_ops(void) {
    return &uart_pio_ops;
}

void uart_pio_init(transport_t *t, PIO pio, uint sm_tx, uint sm_rx,
                   uint pin_tx, uint pin_rx, uint32_t baud) {
    /* Allocate static storage for PIO UART instances */
    static uart_pio_t pio_instances[4];
    static uint8_t pio_count = 0;

    if (pio_count >= 4) {
        return;  /* Max 4 PIO UARTs */
    }

    uart_pio_t *p = &pio_instances[pio_count++];
    p->pio = pio;
    p->sm_tx = sm_tx;
    p->sm_rx = sm_rx;
    p->pin_tx = pin_tx;
    p->pin_rx = pin_rx;

    /* Initialize PIO state machines */
    pio_uart_init(pio, sm_tx, sm_rx, pin_tx, pin_rx, baud);

    t->type = TRANSPORT_UART_PIO;
    t->ops = &uart_pio_ops;
    t->hw = p;
    t->baud = baud;
}