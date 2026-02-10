/**
 * @file uart_hw.c
 * @brief Hardware UART transport implementation
 */

#include "uart_hw.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

/* ──────────────────────────────────────────────
 *  Transport operations
 * ────────────────────────────────────────────── */
static bool uart_hw_write(transport_t *t, const uint8_t *data, size_t len) {
    uart_hw_t *hw = (uart_hw_t *)t->hw;

    if (hw->half_duplex) {
        /* Disable RX to prevent echo */
        gpio_set_function(hw->pin_rx, GPIO_FUNC_NULL);
    }

    uart_write_blocking(hw->uart, data, len);

    if (hw->half_duplex) {
        /* Wait for TX to complete, then re-enable RX */
        uart_tx_wait_blocking(hw->uart);
        gpio_set_function(hw->pin_rx, GPIO_FUNC_UART);
    }

    return true;
}

static size_t uart_hw_read(transport_t *t, uint8_t *buf, size_t max_len) {
    uart_hw_t *hw = (uart_hw_t *)t->hw;
    size_t count = 0;

    while (count < max_len && uart_is_readable(hw->uart)) {
        buf[count++] = uart_getc(hw->uart);
    }

    return count;
}

static bool uart_hw_readable(transport_t *t) {
    uart_hw_t *hw = (uart_hw_t *)t->hw;
    return uart_is_readable(hw->uart);
}

static void uart_hw_flush(transport_t *t) {
    uart_hw_t *hw = (uart_hw_t *)t->hw;
    uart_tx_wait_blocking(hw->uart);
}

static const transport_ops_t uart_hw_ops = {
    .write = uart_hw_write,
    .read = uart_hw_read,
    .readable = uart_hw_readable,
    .flush = uart_hw_flush,
    .set_tx_mode = NULL,  /* Handled internally for half-duplex */
};

/* ──────────────────────────────────────────────
 *  Public API
 * ────────────────────────────────────────────── */
const transport_ops_t *uart_hw_get_ops(void) {
    return &uart_hw_ops;
}

void uart_hw_init(transport_t *t, uart_inst_t *uart,
                  uint pin_tx, uint pin_rx, uint32_t baud, bool half_duplex) {
    static uart_hw_t hw_instances[2];  /* uart0 and uart1 */
    int idx = (uart == uart0) ? 0 : 1;

    uart_hw_t *hw = &hw_instances[idx];
    hw->uart = uart;
    hw->pin_tx = pin_tx;
    hw->pin_rx = pin_rx;
    hw->half_duplex = half_duplex;

    uart_init(uart, baud);
    gpio_set_function(pin_tx, GPIO_FUNC_UART);
    gpio_set_function(pin_rx, GPIO_FUNC_UART);

    t->type = TRANSPORT_UART_HW;
    t->ops = &uart_hw_ops;
    t->hw = hw;
    t->baud = baud;
}