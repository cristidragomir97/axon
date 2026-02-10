/**
 * @file rs485.c
 * @brief RS-485 transport implementation
 */

#include "rs485.h"
#include "pico/stdlib.h"
#include "pio_uart.h"

/* ──────────────────────────────────────────────
 *  Transport operations
 * ────────────────────────────────────────────── */
static void rs485_set_tx_mode(transport_t *t, bool tx) {
    rs485_t *rs = (rs485_t *)t->hw;
    gpio_put(rs->pin_de, tx ? 1 : 0);
}

static bool rs485_write(transport_t *t, const uint8_t *data, size_t len) {
    rs485_t *rs = (rs485_t *)t->hw;

    /* Enable transmitter */
    rs485_set_tx_mode(t, true);

    for (size_t i = 0; i < len; i++) {
        pio_uart_putc(rs->pio, rs->sm_tx, data[i]);
    }

    /* Wait for last byte to transmit (rough estimate) */
    sleep_us((10 * 1000000) / t->baud);

    /* Back to receive mode */
    rs485_set_tx_mode(t, false);

    return true;
}

static size_t rs485_read(transport_t *t, uint8_t *buf, size_t max_len) {
    rs485_t *rs = (rs485_t *)t->hw;
    size_t count = 0;

    while (count < max_len && pio_uart_readable(rs->pio, rs->sm_rx)) {
        int c = pio_uart_getc_nonblock(rs->pio, rs->sm_rx);
        if (c >= 0) {
            buf[count++] = (uint8_t)c;
        }
    }

    return count;
}

static bool rs485_readable(transport_t *t) {
    rs485_t *rs = (rs485_t *)t->hw;
    return pio_uart_readable(rs->pio, rs->sm_rx);
}

static void rs485_flush(transport_t *t) {
    /* Wait for transmission to complete */
    sleep_us((10 * 1000000) / t->baud);
}

static const transport_ops_t rs485_ops = {
    .write = rs485_write,
    .read = rs485_read,
    .readable = rs485_readable,
    .flush = rs485_flush,
    .set_tx_mode = rs485_set_tx_mode,
};

/* ──────────────────────────────────────────────
 *  Public API
 * ────────────────────────────────────────────── */
const transport_ops_t *rs485_get_ops(void) {
    return &rs485_ops;
}

void rs485_init(transport_t *t, PIO pio, uint sm_tx, uint sm_rx,
                uint pin_tx, uint pin_rx, uint pin_de, uint32_t baud) {
    static rs485_t rs485_instance;

    rs485_t *rs = &rs485_instance;
    rs->pio = pio;
    rs->sm_tx = sm_tx;
    rs->sm_rx = sm_rx;
    rs->pin_tx = pin_tx;
    rs->pin_rx = pin_rx;
    rs->pin_de = pin_de;

    /* Initialize DE pin (RX mode by default) */
    gpio_init(pin_de);
    gpio_set_dir(pin_de, GPIO_OUT);
    gpio_put(pin_de, 0);

    /* Initialize PIO UART */
    pio_uart_init(pio, sm_tx, sm_rx, pin_tx, pin_rx, baud);

    t->type = TRANSPORT_RS485;
    t->ops = &rs485_ops;
    t->hw = rs;
    t->baud = baud;
}