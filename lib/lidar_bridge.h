/**
 * @file lidar_bridge.h
 * @brief RPLIDAR C1 passthrough bridge (PIO UART <-> USB CDC)
 *
 * Transparent bridge - MCU does not parse RPLIDAR protocol.
 * rplidar_ros2 on SBC connects to /dev/robocore_lidar
 */

#ifndef LIDAR_BRIDGE_H
#define LIDAR_BRIDGE_H

#include "config.h"

#ifdef ENABLE_LIDAR_BRIDGE

/**
 * @brief Initialize lidar bridge
 *
 * Call after PIO UART is initialized for lidar pins.
 */
void lidar_bridge_init(void);

/**
 * @brief Bridge task - call from Core 1 main loop
 *
 * Transfers data between PIO UART RX and USB CDC TX.
 */
void lidar_bridge_task(void);

/**
 * @brief Handle USB host -> lidar data
 *
 * Called from tud_cdc_rx_cb when data arrives on CDC1.
 */
void lidar_bridge_host_rx(void);

#endif /* ENABLE_LIDAR_BRIDGE */

#endif /* LIDAR_BRIDGE_H */