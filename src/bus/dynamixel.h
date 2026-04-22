#ifndef BUS_DYNAMIXEL_H
#define BUS_DYNAMIXEL_H

#include <stdint.h>
#include <stdbool.h>

// Dynamixel servo bus (half-duplex UART via PIO)
// Pins: TX=GPIO7, RX=GPIO8, TXEN=GPIO17
// Uses PIO0 SM2 (TX) and SM3 (RX)

// Initialize Dynamixel bus
void dynamixel_init(uint32_t baudrate);

// Set baudrate dynamically
void dynamixel_set_baudrate(uint32_t baudrate);

// Send command packet and receive response
// Returns response length, or -1 on timeout/error
int dynamixel_transact(const uint8_t *tx_data, uint32_t tx_len,
                       uint8_t *rx_data, uint32_t rx_max_len,
                       uint32_t timeout_us);

// Low-level: send data only (for broadcast commands)
uint32_t dynamixel_write(const uint8_t *data, uint32_t len);

// Low-level: read available data
uint32_t dynamixel_read(uint8_t *data, uint32_t max_len);

// Check bytes available
uint32_t dynamixel_available(void);

// Poll task - call from main loop
void dynamixel_task(void);

#endif // BUS_DYNAMIXEL_H
