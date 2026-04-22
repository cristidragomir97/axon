#ifndef BUS_UART_BRIDGE_H
#define BUS_UART_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

// UART bridge channels (directly map to CDC ports)
// These use PIO because pins don't align with hardware UART function select
typedef enum {
    UART_BRIDGE_0 = 0,  // GPIO3/4  - general purpose / DDSM
    UART_BRIDGE_1,      // GPIO5/6  - lidar passthrough
#ifdef ENABLE_UART2
    UART_BRIDGE_2,      // GPIO19/20 - additional general purpose
#endif
    UART_BRIDGE_COUNT
} uart_bridge_id_t;

// UART configuration
typedef struct {
    uint32_t baudrate;
    uint8_t  data_bits;  // 5-8
    uint8_t  stop_bits;  // 1-2
    uint8_t  parity;     // 0=none, 1=odd, 2=even
} uart_config_t;

// Default configuration: 115200 8N1
#define UART_CONFIG_DEFAULT { .baudrate = 115200, .data_bits = 8, .stop_bits = 1, .parity = 0 }

// Initialize all UART bridge channels
void uart_bridge_init(void);

// Configure a specific channel
void uart_bridge_configure(uart_bridge_id_t id, const uart_config_t *config);

// Send data on a channel
uint32_t uart_bridge_write(uart_bridge_id_t id, const uint8_t *data, uint32_t len);

// Read data from a channel (non-blocking)
uint32_t uart_bridge_read(uart_bridge_id_t id, uint8_t *data, uint32_t max_len);

// Check bytes available on a channel
uint32_t uart_bridge_available(uart_bridge_id_t id);

// Poll task - call from main loop
void uart_bridge_task(void);

#endif // BUS_UART_BRIDGE_H
