#ifndef LED_H
#define LED_H

#include <stdint.h>
#include <stdbool.h>

// Activity LED identifiers (match pins.h ordering)
typedef enum {
    LED_RS485 = 0,
    LED_DYNAMIXEL,
    LED_FEETECH,
    LED_UART1,
    LED_UART2,
    LED_I2C,
    LED_CAN,
    LED_COUNT
} led_id_t;

// Initialize all activity LEDs
void led_init(void);

// Pulse an LED to indicate activity (non-blocking)
// LED will turn on and auto-off after ~50ms
void led_activity(led_id_t led);

// Set LED state directly
void led_set(led_id_t led, bool on);

// Must be called periodically from main loop (~1ms interval)
void led_task(void);

// Turn all LEDs on or off
void led_all(bool on);

// Flash all LEDs for success indication
void led_flash_all(uint32_t count, uint32_t on_ms, uint32_t off_ms);

#endif // LED_H
