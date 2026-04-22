#ifndef BUS_FEETECH_H
#define BUS_FEETECH_H

#include <stdint.h>
#include <stdbool.h>

// Feetech STS/SCS servo bus (half-duplex UART via PIO)
// Pins: TX=GPIO5, RX=GPIO6, TXEN=GPIO16
// Uses PIO0 SM0 (TX) and SM1 (RX)

// Initialize Feetech bus
void feetech_init(uint32_t baudrate);

// Set baudrate dynamically
void feetech_set_baudrate(uint32_t baudrate);

// Send command packet and receive response
// Returns response length, or -1 on timeout/error
int feetech_transact(const uint8_t *tx_data, uint32_t tx_len,
                     uint8_t *rx_data, uint32_t rx_max_len,
                     uint32_t timeout_us);

// Low-level: send data only (for broadcast commands)
uint32_t feetech_write(const uint8_t *data, uint32_t len);

// Low-level: read available data
uint32_t feetech_read(uint8_t *data, uint32_t max_len);

// Check bytes available
uint32_t feetech_available(void);

// Poll task - call from main loop
void feetech_task(void);

#endif // BUS_FEETECH_H
