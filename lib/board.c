/**
 * @file board.c
 * @brief RoboCore Axon board-level initialization
 */

#include "board.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/adc.h"

#include "transport/uart_hw.h"
#include "transport/uart_pio.h"
#include "transport/rs485.h"

/* ──────────────────────────────────────────────
 *  Bus instances
 * ────────────────────────────────────────────── */
transport_t bus_feetech;
transport_t bus_dynamixel;
transport_t bus_ddsm;
transport_t bus_rs485;

/* ──────────────────────────────────────────────
 *  Internal init functions
 * ────────────────────────────────────────────── */
static void init_gpio(void) {
    /* RS-485 direction control */
    gpio_init(PIN_RS485_DE);
    gpio_set_dir(PIN_RS485_DE, GPIO_OUT);
    gpio_put(PIN_RS485_DE, 0);

    /* mikroBUS control pins */
    gpio_init(PIN_MB_RST);
    gpio_set_dir(PIN_MB_RST, GPIO_OUT);
    gpio_put(PIN_MB_RST, 1);

    gpio_init(PIN_MB_CS);
    gpio_set_dir(PIN_MB_CS, GPIO_OUT);
    gpio_put(PIN_MB_CS, 1);

    gpio_init(PIN_MB_INT);
    gpio_set_dir(PIN_MB_INT, GPIO_IN);
    gpio_pull_up(PIN_MB_INT);

    /* Spare GPIO */
    gpio_init(PIN_SPARE_GPIO);
    gpio_set_dir(PIN_SPARE_GPIO, GPIO_IN);
    gpio_pull_up(PIN_SPARE_GPIO);
}

static void init_i2c(void) {
    i2c_init(SENSOR_I2C, I2C_FREQ_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
}

static void init_spi(void) {
    spi_init(MB_SPI, 1000000);
    gpio_set_function(PIN_MB_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MB_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MB_MISO, GPIO_FUNC_SPI);
}

static void init_adc(void) {
    adc_init();
    adc_gpio_init(PIN_MB_AN);
    adc_gpio_init(PIN_SPARE_ADC1);
    adc_gpio_init(PIN_SPARE_ADC2);
}

static void init_buses(void) {
    /* Feetech bus: HW UART1, half-duplex, 1Mbps */
    uart_hw_init(&bus_feetech, ST_UART, PIN_ST_TX, PIN_ST_RX, ST_BAUD, true);

    /* Dynamixel bus: HW UART0, half-duplex, 57600 */
    uart_hw_init(&bus_dynamixel, DX_UART, PIN_DX_TX, PIN_DX_RX, DX_BAUD, true);

    /* DDSM210 bus: PIO UART, full-duplex */
    uart_pio_init(&bus_ddsm, DDS_PIO, DDS_SM_TX, DDS_SM_RX,
                  PIN_DDS_TX, PIN_DDS_RX, DDS_BAUD);

    /* RS-485 bus: PIO UART with DE control */
    rs485_init(&bus_rs485, RS485_PIO, RS485_SM_TX, RS485_SM_RX,
               PIN_RS485_TX, PIN_RS485_RX, PIN_RS485_DE, RS485_BAUD);
}

/* ──────────────────────────────────────────────
 *  Public API
 * ────────────────────────────────────────────── */
void board_init(void) {
    init_gpio();
    init_i2c();
    init_spi();
    init_adc();
    init_buses();
}
