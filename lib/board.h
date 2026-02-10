/**
 * @file board.h
 * @brief RoboCore Axon board-level configuration
 *
 * Defines transports and hardware initialization.
 * This is board-specific, not user-configurable.
 */

#ifndef BOARD_H
#define BOARD_H

#include "pinout.h"
#include "transport/transport.h"

/* ──────────────────────────────────────────────
 *  Available buses (transport instances)
 *
 *  These names are referenced in config/motors.yaml
 * ────────────────────────────────────────────── */
extern transport_t bus_feetech;    /* HW UART1, half-duplex, 1Mbps */
extern transport_t bus_dynamixel;  /* HW UART0, half-duplex, 57600 */
extern transport_t bus_ddsm;       /* PIO UART, full-duplex, 115200 */
extern transport_t bus_rs485;      /* PIO UART + DE, 115200 */

/**
 * @brief Initialize all board hardware
 *
 * Call once at startup before motor_init_all().
 * Initializes GPIO, I2C, SPI, ADC, and all motor buses.
 */
void board_init(void);

#endif /* BOARD_H */
