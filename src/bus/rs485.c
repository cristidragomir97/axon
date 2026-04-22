#include "rs485.h"
#include "pins.h"
#include "led_dual_core.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define RS485_UART uart0

// Ring buffer for RX (simple implementation)
#define RS485_RX_BUF_SIZE 256
static uint8_t rx_buf[RS485_RX_BUF_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;

static bool initialized = false;

void rs485_init(uint32_t baudrate) {
    // Initialize DE pin first (ensure we're in RX mode)
    gpio_init(PIN_RS485_DE);
    gpio_set_dir(PIN_RS485_DE, GPIO_OUT);
    gpio_put(PIN_RS485_DE, 0);  // DE low = receive mode

    // Initialize UART
    uart_init(RS485_UART, baudrate);
    gpio_set_function(PIN_RS485_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RS485_RX, GPIO_FUNC_UART);

    // Configure UART parameters: 8N1
    uart_set_format(RS485_UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(RS485_UART, false, false);
    uart_set_fifo_enabled(RS485_UART, true);

    rx_head = rx_tail = 0;
    initialized = true;
}

void rs485_set_baudrate(uint32_t baudrate) {
    if (!initialized) return;
    uart_set_baudrate(RS485_UART, baudrate);
}

uint32_t rs485_write(const uint8_t *data, uint32_t len) {
    if (!initialized || len == 0) return 0;

    // Assert DE (transmit mode)
    gpio_put(PIN_RS485_DE, 1);

    // Small delay for transceiver to switch
    busy_wait_us(5);

    // Send data
    for (uint32_t i = 0; i < len; i++) {
        uart_putc_raw(RS485_UART, data[i]);
    }

    // Wait for TX to complete
    uart_tx_wait_blocking(RS485_UART);

    // Small delay before switching back
    busy_wait_us(5);

    // Deassert DE (receive mode)
    gpio_put(PIN_RS485_DE, 0);

    led_activity(LED_RS485);
    return len;
}

uint32_t rs485_read(uint8_t *data, uint32_t max_len) {
    if (!initialized) return 0;

    uint32_t count = 0;
    while (count < max_len && rx_head != rx_tail) {
        data[count++] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RS485_RX_BUF_SIZE;
    }

    if (count > 0) {
        led_activity(LED_RS485);
    }
    return count;
}

uint32_t rs485_available(void) {
    if (!initialized) return 0;
    return (rx_head - rx_tail + RS485_RX_BUF_SIZE) % RS485_RX_BUF_SIZE;
}

void rs485_flush(void) {
    if (!initialized) return;
    uart_tx_wait_blocking(RS485_UART);
}

void rs485_task(void) {
    if (!initialized) return;

    // Poll UART RX and fill ring buffer
    while (uart_is_readable(RS485_UART)) {
        uint8_t c = uart_getc(RS485_UART);
        uint32_t next_head = (rx_head + 1) % RS485_RX_BUF_SIZE;
        if (next_head != rx_tail) {  // Not full
            rx_buf[rx_head] = c;
            rx_head = next_head;
        }
        // If full, drop the byte
    }
}
