#include "uart_bridge.h"
#include "pins.h"
#include "led_dual_core.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <stdio.h>

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

// PIO UART helper functions
static inline void pio_uart_tx_program_putc(PIO pio, uint sm, char c) {
    pio_sm_put_blocking(pio, sm, (uint32_t)c);
}

static inline bool pio_uart_tx_program_is_busy(PIO pio, uint sm) {
    return !pio_sm_is_tx_fifo_empty(pio, sm);
}

static inline uint8_t pio_uart_rx_program_getc(PIO pio, uint sm) {
    return (uint8_t)(pio_sm_get(pio, sm) >> 24);
}

// Uses PIO1 (PIO0 used by Feetech/Dynamixel)
#define UART_BRIDGE_PIO pio1
#ifdef ENABLE_UART2
// Also uses PIO2 for optional UART2 (shares with LEDs on Core 1)
#define UART2_PIO pio2
#endif

// State machine assignments
// PIO1:
// SM0: UART0 TX (GPIO3)
// SM1: UART0 RX (GPIO4)
// SM2: UART1 TX (GPIO5)
// SM3: UART1 RX (GPIO6)
//
// PIO2 (if ENABLE_UART2):
// SM0: NeoPixel (Core 1)
// SM1: UART2 TX (GPIO19)
// SM2: UART2 RX (GPIO20)
// SM3: free

typedef struct {
    uint8_t tx_pin;
    uint8_t rx_pin;
    uint8_t sm_tx;
    uint8_t sm_rx;
    led_id_t led;
} uart_channel_config_t;

static const uart_channel_config_t channel_configs[UART_BRIDGE_COUNT] = {
    [UART_BRIDGE_0] = {
        .tx_pin = PIN_UART0_TX,
        .rx_pin = PIN_UART0_RX,
        .sm_tx = 0,
        .sm_rx = 1,
        .led = LED_UART0
    },
    [UART_BRIDGE_1] = {
        .tx_pin = PIN_UART1_TX,
        .rx_pin = PIN_UART1_RX,
        .sm_tx = 2,
        .sm_rx = 3,
        .led = LED_UART1
    },
#ifdef ENABLE_UART2
    [UART_BRIDGE_2] = {
        .tx_pin = PIN_UART2_TX,
        .rx_pin = PIN_UART2_RX,
        .sm_tx = 1,  // PIO2 SM1
        .sm_rx = 2,  // PIO2 SM2
        .led = LED_UART1  // Share LED with UART1 (only 6 physical LEDs available)
    },
#endif
};

#define UART_RX_BUF_SIZE 512

typedef struct {
    uint8_t buffer[UART_RX_BUF_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    bool initialized;
    uint32_t baudrate;
} uart_channel_state_t;

static uart_channel_state_t channel_state[UART_BRIDGE_COUNT];
static uint tx_offset = 0;
static uint rx_offset = 0;
#ifdef ENABLE_UART2
static uint tx_offset_pio2 = 0;
static uint rx_offset_pio2 = 0;
#endif
static bool programs_loaded = false;

// Helper function to get the correct PIO for a channel
static inline PIO get_channel_pio(uart_bridge_id_t id) {
#ifdef ENABLE_UART2
    if (id == UART_BRIDGE_2) return UART2_PIO;
#endif
    return UART_BRIDGE_PIO;
}

// Helper function to get the correct program offsets for a channel
static inline void get_channel_offsets(uart_bridge_id_t id, uint *tx_off, uint *rx_off) {
#ifdef ENABLE_UART2
    if (id == UART_BRIDGE_2) {
        *tx_off = tx_offset_pio2;
        *rx_off = rx_offset_pio2;
        return;
    }
#endif
    *tx_off = tx_offset;
    *rx_off = rx_offset;
}

void uart_bridge_init(void) {
    // Load PIO programs once
    if (!programs_loaded) {
        tx_offset = pio_add_program(UART_BRIDGE_PIO, &pio_uart_tx_program);
        rx_offset = pio_add_program(UART_BRIDGE_PIO, &pio_uart_rx_program);
#ifdef ENABLE_UART2
        tx_offset_pio2 = pio_add_program(UART2_PIO, &pio_uart_tx_program);
        rx_offset_pio2 = pio_add_program(UART2_PIO, &pio_uart_rx_program);
        printf("[UART Bridge] Loaded PIO programs: PIO1 TX@%u RX@%u, PIO2 TX@%u RX@%u\n", 
               tx_offset, rx_offset, tx_offset_pio2, rx_offset_pio2);
#else
        printf("[UART Bridge] Loaded PIO programs: TX@%u RX@%u\n", tx_offset, rx_offset);
#endif
        programs_loaded = true;
    }

    for (int i = 0; i < UART_BRIDGE_COUNT; i++) {
        const uart_channel_config_t *cfg = &channel_configs[i];
        uint32_t baudrate = 115200;  // Default
        PIO pio = get_channel_pio(i);
        uint tx_off, rx_off;
        get_channel_offsets(i, &tx_off, &rx_off);

        pio_uart_tx_program_init(pio, cfg->sm_tx, tx_off, cfg->tx_pin, baudrate);
        pio_uart_rx_program_init(pio, cfg->sm_rx, rx_off, cfg->rx_pin, baudrate);

        channel_state[i].head = 0;
        channel_state[i].tail = 0;
        channel_state[i].baudrate = baudrate;
        channel_state[i].initialized = true;
        
        printf("[UART Bridge] Channel %d: TX pin %d (SM%d), RX pin %d (SM%d) @ %lu baud\n", 
               i, cfg->tx_pin, cfg->sm_tx, cfg->rx_pin, cfg->sm_rx, baudrate);
    }
}

void uart_bridge_configure(uart_bridge_id_t id, const uart_config_t *config) {
    if (id >= UART_BRIDGE_COUNT || !config) return;
    if (!channel_state[id].initialized) return;
    if (config->baudrate == channel_state[id].baudrate) return;

    const uart_channel_config_t *cfg = &channel_configs[id];
    PIO pio = get_channel_pio(id);

    pio_sm_set_enabled(pio, cfg->sm_tx, false);
    pio_sm_set_enabled(pio, cfg->sm_rx, false);

    float div = (float)clock_get_hz(clk_sys) / (config->baudrate * 8);
    pio_sm_clear_fifos(pio, cfg->sm_tx);
    pio_sm_set_clkdiv(pio, cfg->sm_tx, div);
    pio_sm_clear_fifos(pio, cfg->sm_rx);
    pio_sm_set_clkdiv(pio, cfg->sm_rx, div);

    pio_sm_set_enabled(pio, cfg->sm_tx, true);
    pio_sm_set_enabled(pio, cfg->sm_rx, true);

    channel_state[id].baudrate = config->baudrate;
}

uint32_t uart_bridge_write(uart_bridge_id_t id, const uint8_t *data, uint32_t len) {
    if (id >= UART_BRIDGE_COUNT || !channel_state[id].initialized) return 0;
    if (len == 0) return 0;

    const uart_channel_config_t *cfg = &channel_configs[id];
    PIO pio = get_channel_pio(id);

    for (uint32_t i = 0; i < len; i++) {
        pio_uart_tx_program_putc(pio, cfg->sm_tx, data[i]);
    }

    while (pio_uart_tx_program_is_busy(pio, cfg->sm_tx)) {
        tight_loop_contents();
    }

    led_activity(cfg->led);
    return len;
}

uint32_t uart_bridge_read(uart_bridge_id_t id, uint8_t *data, uint32_t max_len) {
    if (id >= UART_BRIDGE_COUNT || !channel_state[id].initialized) return 0;

    uart_channel_state_t *state = &channel_state[id];
    const uart_channel_config_t *cfg = &channel_configs[id];

    uint32_t count = 0;
    while (count < max_len && state->head != state->tail) {
        data[count++] = state->buffer[state->tail];
        state->tail = (state->tail + 1) % UART_RX_BUF_SIZE;
    }

    if (count > 0) led_activity(cfg->led);
    return count;
}

uint32_t uart_bridge_available(uart_bridge_id_t id) {
    if (id >= UART_BRIDGE_COUNT || !channel_state[id].initialized) return 0;
    uart_channel_state_t *state = &channel_state[id];
    return (state->head - state->tail + UART_RX_BUF_SIZE) % UART_RX_BUF_SIZE;
}

void uart_bridge_task(void) {
    for (int i = 0; i < UART_BRIDGE_COUNT; i++) {
        if (!channel_state[i].initialized) continue;

        const uart_channel_config_t *cfg = &channel_configs[i];
        uart_channel_state_t *state = &channel_state[i];
        PIO pio = get_channel_pio(i);

        while (!pio_sm_is_rx_fifo_empty(pio, cfg->sm_rx)) {
            uint8_t c = (uint8_t)(pio_sm_get(pio, cfg->sm_rx) >> 24);
            uint32_t next_head = (state->head + 1) % UART_RX_BUF_SIZE;
            if (next_head != state->tail) {
                state->buffer[state->head] = c;
                state->head = next_head;
            }
        }
    }
}
