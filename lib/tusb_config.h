/**
 * @file tusb_config.h
 * @brief TinyUSB configuration for dual CDC composite device
 */

#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#include "config.h"

/* ──────────────────────────────────────────────
 *  Board / MCU
 * ────────────────────────────────────────────── */
#define CFG_TUSB_MCU                OPT_MCU_RP2040  /* Works for RP2350 too */
#define CFG_TUSB_OS                 OPT_OS_PICO
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE

/* ──────────────────────────────────────────────
 *  Device configuration
 * ────────────────────────────────────────────── */
#define CFG_TUD_ENABLED             1
#define CFG_TUD_MAX_SPEED           OPT_MODE_FULL_SPEED

/* Endpoint 0 size */
#define CFG_TUD_ENDPOINT0_SIZE      64

/* ──────────────────────────────────────────────
 *  Class drivers
 * ────────────────────────────────────────────── */
#ifdef ENABLE_LIDAR_BRIDGE
#define CFG_TUD_CDC                 2   /* CDC0: zenoh, CDC1: lidar */
#else
#define CFG_TUD_CDC                 1   /* CDC0: zenoh only */
#endif

#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

/* ──────────────────────────────────────────────
 *  CDC FIFO sizes
 * ────────────────────────────────────────────── */
#define CFG_TUD_CDC_RX_BUFSIZE      256
#define CFG_TUD_CDC_TX_BUFSIZE      256

/* ──────────────────────────────────────────────
 *  Memory
 * ────────────────────────────────────────────── */
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN          __attribute__((aligned(4)))

#endif /* TUSB_CONFIG_H */