/**
 * @file feetech.h
 * @brief Feetech SCS/STS servo protocol
 *
 * Supports SCS and STS series servos (e.g., STS3215).
 * Half-duplex single-wire protocol at 1Mbps.
 */

#ifndef FEETECH_H
#define FEETECH_H

#include "motor.h"
#include "transport/transport.h"

/* ──────────────────────────────────────────────
 *  Protocol constants
 * ────────────────────────────────────────────── */
#define FEETECH_HEADER          0xFF
#define FEETECH_BROADCAST_ID    0xFE

/* Instructions */
#define FEETECH_INST_PING       0x01
#define FEETECH_INST_READ       0x02
#define FEETECH_INST_WRITE      0x03
#define FEETECH_INST_REG_WRITE  0x04
#define FEETECH_INST_ACTION     0x05
#define FEETECH_INST_SYNC_READ  0x82
#define FEETECH_INST_SYNC_WRITE 0x83

/* Common registers (STS3215) */
#define FEETECH_REG_ID              5
#define FEETECH_REG_BAUD            6
#define FEETECH_REG_MIN_ANGLE_L     9
#define FEETECH_REG_MAX_ANGLE_L     11
#define FEETECH_REG_MODE            33
#define FEETECH_REG_TORQUE_ENABLE   40
#define FEETECH_REG_GOAL_POS_L      42
#define FEETECH_REG_GOAL_TIME_L     44
#define FEETECH_REG_GOAL_SPEED_L    46
#define FEETECH_REG_LOCK            55
#define FEETECH_REG_PRESENT_POS_L   56
#define FEETECH_REG_PRESENT_SPEED_L 58
#define FEETECH_REG_PRESENT_LOAD_L  60
#define FEETECH_REG_PRESENT_VOLTAGE 62
#define FEETECH_REG_PRESENT_TEMP    63
#define FEETECH_REG_MOVING          66
#define FEETECH_REG_PRESENT_CURRENT_L 69

/* Mode values */
#define FEETECH_MODE_SERVO      0
#define FEETECH_MODE_WHEEL      1
#define FEETECH_MODE_STEP       3

/* ──────────────────────────────────────────────
 *  Feetech-specific motor data
 * ────────────────────────────────────────────── */
typedef struct {
    uint16_t model;
    uint16_t pos_offset;      /* Zero position offset (raw units) */
    float pos_scale;          /* Raw units per radian */
    float vel_scale;          /* Raw units per rad/s */
    float torque_scale;       /* Raw units per Nm */
    uint32_t last_poll_us;    /* Timestamp of last poll */
} feetech_data_t;

/**
 * @brief Get Feetech motor operations
 */
const motor_ops_t *feetech_get_ops(void);

/**
 * @brief Create a Feetech motor instance
 *
 * @param motor Motor struct to initialize
 * @param transport Transport to use
 * @param id Servo ID (1-253)
 * @param name Joint name
 */
void feetech_motor_init(motor_t *motor, transport_t *transport,
                        uint8_t id, const char *name);

#endif /* FEETECH_H */