/**
 * @file lidar_bridge.c
 * @brief RPLIDAR C1 passthrough bridge implementation
 */

#include "config.h"

#ifdef ENABLE_LIDAR_BRIDGE

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "tusb.h"

#include "pinout.h"
#include "pio_uart.h"
#include "lidar_bridge.h"

/* ──────────────────────────────────────────────
 *  Ring buffer
 * ────────────────────────────────────────────── */
#define RING_MASK (LIDAR_BRIDGE_BUF_SIZE - 1)

typedef struct {
    uint8_t buf[LIDAR_BRIDGE_BUF_SIZE];
    volatile uint32_t head;  /* Write index */
    volatile uint32_t tail;  /* Read index */
} ring_buf_t;

static ring_buf_t rx_ring;  /* Lidar -> USB */
static ring_buf_t tx_ring;  /* USB -> Lidar */

static inline void ring_init(ring_buf_t *r) {
    r->head = 0;
    r->tail = 0;
}

static inline uint32_t ring_count(ring_buf_t *r) {
    return (r->head - r->tail) & RING_MASK;
}

static inline uint32_t ring_free(ring_buf_t *r) {
    return LIDAR_BRIDGE_BUF_SIZE - 1 - ring_count(r);
}

static inline bool ring_empty(ring_buf_t *r) {
    return r->head == r->tail;
}

static inline bool ring_put(ring_buf_t *r, uint8_t c) {
    if (ring_free(r) == 0) return false;
    r->buf[r->head & RING_MASK] = c;
    r->head++;
    return true;
}

static inline int ring_get(ring_buf_t *r) {
    if (ring_empty(r)) return -1;
    uint8_t c = r->buf[r->tail & RING_MASK];
    r->tail++;
    return c;
}

/* ──────────────────────────────────────────────
 *  Bridge implementation
 * ────────────────────────────────────────────── */
void lidar_bridge_init(void) {
    ring_init(&rx_ring);
    ring_init(&tx_ring);
}

void lidar_bridge_task(void) {
    /* PIO RX -> ring buffer (lidar data coming in) */
    while (pio_uart_readable(LIDAR_PIO, LIDAR_SM_RX)) {
        int c = pio_uart_getc_nonblock(LIDAR_PIO, LIDAR_SM_RX);
        if (c >= 0) {
            ring_put(&rx_ring, (uint8_t)c);
        }
    }

    /* Ring buffer -> USB CDC TX */
    if (tud_cdc_n_connected(USB_CDC_LIDAR) && tud_cdc_n_write_available(USB_CDC_LIDAR)) {
        uint8_t tmp[64];
        uint32_t count = 0;

        while (count < sizeof(tmp) && !ring_empty(&rx_ring)) {
            int c = ring_get(&rx_ring);
            if (c >= 0) {
                tmp[count++] = (uint8_t)c;
            }
        }

        if (count > 0) {
            tud_cdc_n_write(USB_CDC_LIDAR, tmp, count);
            tud_cdc_n_write_flush(USB_CDC_LIDAR);
        }
    }

    /* TX ring buffer -> PIO TX (commands to lidar) */
    while (!ring_empty(&tx_ring)) {
        int c = ring_get(&tx_ring);
        if (c >= 0) {
            pio_uart_putc(LIDAR_PIO, LIDAR_SM_TX, (char)c);
        }
    }
}

void lidar_bridge_host_rx(void) {
    uint8_t buf[64];
    uint32_t count = tud_cdc_n_read(USB_CDC_LIDAR, buf, sizeof(buf));

    for (uint32_t i = 0; i < count; i++) {
        ring_put(&tx_ring, buf[i]);
    }
}

#endif /* ENABLE_LIDAR_BRIDGE */