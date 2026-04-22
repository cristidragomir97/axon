#ifndef BUS_RS485_H
#define BUS_RS485_H

#include <stdint.h>
#include <stdbool.h>

// RS485 bus using hardware UART0 with direction enable (DE) control
// Pins: TX=GPIO0, RX=GPIO1, DE=GPIO2 (directly compatible with UART0 func select)

// Initialize RS485 hardware
void rs485_init(uint32_t baudrate);

// Set baudrate dynamically
void rs485_set_baudrate(uint32_t baudrate);

// Send data (handles DE assertion automatically)
// Returns number of bytes sent
uint32_t rs485_write(const uint8_t *data, uint32_t len);

// Read available data (non-blocking)
// Returns number of bytes read
uint32_t rs485_read(uint8_t *data, uint32_t max_len);

// Check how many bytes available to read
uint32_t rs485_available(void);

// Flush TX buffer (blocking wait for completion)
void rs485_flush(void);

// Poll task - call from main loop
void rs485_task(void);

#endif // BUS_RS485_H
