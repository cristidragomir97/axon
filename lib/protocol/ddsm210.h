/**
 * @file ddsm210.h
 * @brief Waveshare DDSM210 brushless motor protocol
 *
 * DDSM210 is a 210RPM brushless DC motor with integrated driver.
 * Communication: 115200 baud, 8N1, 10-byte packets with CRC-8/MAXIM.
 *
 * Velocity range: -210 to +210 RPM (0.1 RPM resolution)
 * Position range: 0-32767 (0-360 degrees)
 */

#ifndef DDSM210_H
#define DDSM210_H

#include "motor.h"
#include "transport/transport.h"

/* ──────────────────────────────────────────────
 *  Protocol constants
 * ────────────────────────────────────────────── */
#define DDSM210_PACKET_LEN      10
#define DDSM210_BROADCAST_ID    0x00

/* Command codes */
#define DDSM210_CMD_DRIVE       0x64    /* Motor rotation control */
#define DDSM210_CMD_QUERY_POS   0x74    /* Query mileage and position */
#define DDSM210_CMD_SET_MODE    0xA0    /* Mode switching */
#define DDSM210_CMD_QUERY_MODE  0x75    /* Query current mode */

/* Mode values for CMD_SET_MODE */
#define DDSM210_MODE_OPEN_LOOP  0x00
#define DDSM210_MODE_VELOCITY   0x02
#define DDSM210_MODE_POSITION   0x03

/* Feedback content selectors for CMD_DRIVE */
#define DDSM210_FB_VELOCITY     0x01    /* Current velocity */
#define DDSM210_FB_POSITION     0x02    /* Current position */
#define DDSM210_FB_CURRENT      0x03    /* Current draw */

/* Conversion factors */
#define DDSM210_RPM_SCALE       10.0f   /* Raw units to 0.1 RPM */
#define DDSM210_POS_SCALE       (32768.0f / 360.0f)  /* Raw to degrees */
#define DDSM210_RAD_TO_RPM      (60.0f / (2.0f * 3.14159265f))
#define DDSM210_RPM_TO_RAD      ((2.0f * 3.14159265f) / 60.0f)

/* ──────────────────────────────────────────────
 *  DDSM210-specific motor data
 * ────────────────────────────────────────────── */
typedef struct {
    uint8_t mode;               /* Current operating mode */
    int32_t mileage_laps;       /* Total laps (from query) */
    uint16_t raw_position;      /* Raw position 0-32767 */
    int16_t raw_velocity;       /* Raw velocity in 0.1 RPM */
    uint8_t temperature;        /* Temperature in Celsius */
    uint8_t error_code;         /* Error flags */
    uint32_t last_poll_us;      /* Timestamp of last poll */
    uint32_t last_cmd_us;       /* Timestamp of last command */
} ddsm210_data_t;

/**
 * @brief Get DDSM210 motor operations
 */
const motor_ops_t *ddsm210_get_ops(void);

/**
 * @brief Create a DDSM210 motor instance
 *
 * @param motor Motor struct to initialize
 * @param transport Transport to use (PIO UART at 115200)
 * @param id Motor ID (1-253)
 * @param name Joint name
 */
void ddsm210_motor_init(motor_t *motor, transport_t *transport,
                        uint8_t id, const char *name);

/**
 * @brief Set DDSM210 operating mode
 *
 * @param motor Motor instance
 * @param mode DDSM210_MODE_VELOCITY or DDSM210_MODE_POSITION
 * @return true on success
 */
bool ddsm210_set_mode(motor_t *motor, uint8_t mode);

/**
 * @brief Query mileage and absolute position
 *
 * @param motor Motor instance
 * @param laps Output: total laps traveled
 * @param position Output: current position 0-32767
 * @return true on success
 */
bool ddsm210_query_position(motor_t *motor, int32_t *laps, uint16_t *position);

#endif /* DDSM210_H */
