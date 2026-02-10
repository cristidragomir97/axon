/**
 * @file config.h
 * @brief RoboCore Axon build configuration
 *
 * Comment/uncomment defines to enable/disable features.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ──────────────────────────────────────────────
 *  Feature flags
 * ────────────────────────────────────────────── */
#define ENABLE_LIDAR_BRIDGE     /* USB CDC passthrough for RPLIDAR */

/* Future features (uncomment when implemented):
 * #define ENABLE_RS485_BRIDGE
 * #define ENABLE_DDSM210
 * #define ENABLE_FEETECH
 * #define ENABLE_DYNAMIXEL
 * #define ENABLE_PICOROS
 */

#endif /* CONFIG_H */