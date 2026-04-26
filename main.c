/*
 * Axon Multiprotocol USB Bridge
 * =============================
 *
 * A USB composite device that bridges multiple communication protocols:
 *
 *   USB Interface          Protocol          Hardware
 *   ─────────────────────────────────────────────────────
 *   CDC #0                 RS485             MAX3485
 *   CDC #1                 Feetech STS/SCS   Half-duplex servo bus
 *   CDC #2                 Dynamixel         Half-duplex servo bus
 *   CDC #3                 UART0             General purpose (DDSM)
 *   CDC #4                 UART1             General purpose (Lidar)
 *   Vendor (gs_usb)        CAN               MCP2518FD
 *   Vendor (i2c-tiny-usb)  I2C               Qwiic/Stemma QT
 *
 * Each CDC port appears as a virtual serial port on the host.
 * CAN works with Linux SocketCAN (candleLight/gs_usb compatible).
 * I2C works with Linux i2c-tiny-usb driver.
 *
 * Pin assignments are defined in pins.h
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tusb.h"

#include "pins.h"
#include "usb_descriptors.h"
#include "led_dual_core.h"
#include "usb_handlers.h"
#include "pico/multicore.h"
#include "bus/rs485.h"
#include "bus/half_duplex.h"
#include "bus/can.h"
#include "bus/uart_bridge.h"

//--------------------------------------------------------------------
// Configuration
//--------------------------------------------------------------------
#define I2C_BAUDRATE 400000

//--------------------------------------------------------------------
// CDC Bridge Helper
//
// Bridges data between a USB CDC port and a serial bus.
// Handles bidirectional transfer: USB <-> Hardware
//--------------------------------------------------------------------
static void cdc_bridge(
    uint8_t cdc_idx,
    uint32_t (*bus_write)(const uint8_t *, uint32_t),
    uint32_t (*bus_read)(uint8_t *, uint32_t),
    uint32_t (*bus_available)(void))
{
    uint8_t buf[64];

    // USB -> Bus
    if (tud_cdc_n_available(cdc_idx)) {
        uint32_t count = tud_cdc_n_read(cdc_idx, buf, sizeof(buf));
        if (count > 0) bus_write(buf, count);
    }

    // Bus -> USB
    if (bus_available() > 0 && tud_cdc_n_write_available(cdc_idx) > 0) {
        uint32_t count = bus_read(buf, sizeof(buf));
        if (count > 0) {
            tud_cdc_n_write(cdc_idx, buf, count);
            tud_cdc_n_write_flush(cdc_idx);
        }
    }
}

//--------------------------------------------------------------------
// UART Bridge Helper (uses channel ID)
//--------------------------------------------------------------------
static uint32_t uart0_write(const uint8_t *d, uint32_t n) { return uart_bridge_write(UART_BRIDGE_0, d, n); }
static uint32_t uart0_read(uint8_t *d, uint32_t n) { return uart_bridge_read(UART_BRIDGE_0, d, n); }
static uint32_t uart0_available(void) { return uart_bridge_available(UART_BRIDGE_0); }

static uint32_t uart1_write(const uint8_t *d, uint32_t n) { return uart_bridge_write(UART_BRIDGE_1, d, n); }
static uint32_t uart1_read(uint8_t *d, uint32_t n) { return uart_bridge_read(UART_BRIDGE_1, d, n); }
static uint32_t uart1_available(void) { return uart_bridge_available(UART_BRIDGE_1); }

#ifdef ENABLE_UART2
static uint32_t uart2_write(const uint8_t *d, uint32_t n) { return uart_bridge_write(UART_BRIDGE_2, d, n); }
static uint32_t uart2_read(uint8_t *d, uint32_t n) { return uart_bridge_read(UART_BRIDGE_2, d, n); }
static uint32_t uart2_available(void) { return uart_bridge_available(UART_BRIDGE_2); }
#endif

//--------------------------------------------------------------------
// Unified Motor Bridge Helper
// Uses Feetech as primary interface since both protocols share the same hardware
//--------------------------------------------------------------------
static uint32_t motor_write(const uint8_t *d, uint32_t n) { return half_duplex_write(d, n); }
static uint32_t motor_read(uint8_t *d, uint32_t n) { return half_duplex_read(d, n); }
static uint32_t motor_available(void) { return half_duplex_available(); }

//--------------------------------------------------------------------
// CDC Line Coding Callback
//
// Called when the host changes baud rate on a CDC port.
// We pass this through to the corresponding hardware.
//--------------------------------------------------------------------
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding) {
    switch (itf) {
    case CDC_IDX_RS485:    rs485_set_baudrate(coding->bit_rate); break;
    case CDC_IDX_MOTOR:
        half_duplex_set_baudrate(coding->bit_rate);
        break;
    case CDC_IDX_UART0: {
        uart_config_t cfg = { .baudrate = coding->bit_rate };
        uart_bridge_configure(UART_BRIDGE_0, &cfg);
        break;
    }
    case CDC_IDX_UART1: {
        uart_config_t cfg = { .baudrate = coding->bit_rate };
        uart_bridge_configure(UART_BRIDGE_1, &cfg);
        break;
    }
#ifdef ENABLE_UART2
    case CDC_IDX_UART2: {
        uart_config_t cfg = { .baudrate = coding->bit_rate };
        uart_bridge_configure(UART_BRIDGE_2, &cfg);
        break;
    }
#endif
    }
}

//--------------------------------------------------------------------
// Main
//--------------------------------------------------------------------
int main(void) {
    stdio_init_all();

    // Initialize activity LEDs first
    led_init_dual_core(); multicore_launch_core1(led_core1_main); sleep_ms(100);
    
    // Show we're starting initialization
    led_all(false);
    sleep_ms(200);

    // Initialize I2C (for i2c-tiny-usb) - use i2c1 at 100kHz for better compatibility
    i2c_init(i2c1, 100000);  // 100kHz instead of 400kHz
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    
    // I2C initialized - turn on I2C LED
    led_set(LED_I2C, true);
    sleep_ms(300);

    // Initialize RS485 bus
    rs485_init(115200);
    led_set(LED_RS485, true);
    sleep_ms(300);

    // Initialize unified half-duplex motor bus (Feetech/Dynamixel share the same wire)
    half_duplex_init(1000000);
    led_set(LED_FEETECH, true);
    sleep_ms(300);

    // Initialize UART bridges
    uart_bridge_init();
    led_set(LED_UART0, true);
    sleep_ms(200);
    led_set(LED_UART1, true);
    sleep_ms(300);

    // Initialize CAN controller
    bool can_ok = can_init();
    if (can_ok) {
        led_set(LED_CAN, true);
        printf("[CAN] MCP2518FD initialized\n");
    } else {
        // Flash CAN LED to indicate failure
        for (int i = 0; i < 3; i++) {
            led_set(LED_CAN, true);
            sleep_ms(100);
            led_set(LED_CAN, false);
            sleep_ms(100);
        }
        printf("[CAN] MCP2518FD not detected\n");
    }
    sleep_ms(300);

    // Initialize USB
    usb_handlers_init();
    tusb_init();
    
    // Wait for USB to be ready and enumerate
    printf("Waiting for USB connection...\n");
    uint32_t usb_start = time_us_32();
    while (!tud_ready() && (time_us_32() - usb_start) < 5000000) {  // 5 second timeout
        tud_task();
        sleep_ms(10);
    }
    
    // Final success sequence - flash all LEDs 3 times
    if (tud_ready()) {
        printf("USB enumerated successfully\n");
        led_flash_all(3, 150, 150);
        sleep_ms(500);
        
        // Clear all LEDs - normal operation will light them as needed
        led_all(false);
        sleep_ms(200);
        
        // Brief "ready" pattern - quick sweep
        for (int i = 0; i < LED_COUNT; i++) {
            led_set(i, true);
            sleep_ms(50);
            led_set(i, false);
        }
        sleep_ms(200);
        
        // Final double flash to confirm ready
        led_flash_all(2, 100, 100);
        
    } else {
        printf("USB enumeration timeout\n");
        // Rapid flash to indicate USB failure
        led_flash_all(5, 50, 50);
        sleep_ms(200);
    }

    printf("\n");
    printf("Axon Multiprotocol USB Bridge v2.0\n");
    printf("──────────────────────────────────\n");
    printf("CDC #0: RS485      CDC #2: UART0\n");
    printf("CDC #1: Motor      CDC #3: UART1\n");
    printf("        (Feetech/Dynamixel)\n");
    printf("CAN: gs_usb        I2C: i2c-tiny-usb\n");
    printf("\n");

    // Main loop
    while (true) {
        // Process USB events
        tud_task();

        // Poll hardware drivers (move data from hardware to buffers)
        rs485_task();
        half_duplex_task();
        uart_bridge_task();
        can_task();

        // Bridge CDC ports to their hardware
        cdc_bridge(CDC_IDX_RS485, rs485_write, rs485_read, rs485_available);
        cdc_bridge(CDC_IDX_MOTOR, motor_write, motor_read, motor_available);
        cdc_bridge(CDC_IDX_UART0, uart0_write, uart0_read, uart0_available);
        cdc_bridge(CDC_IDX_UART1, uart1_write, uart1_read, uart1_available);
#ifdef ENABLE_UART2
        cdc_bridge(CDC_IDX_UART2, uart2_write, uart2_read, uart2_available);
#endif

        // Bridge CAN to USB (gs_usb protocol)
        can_usb_task();

        // LED updates are handled by Core 1 - no need to call led_task()
    }
}
