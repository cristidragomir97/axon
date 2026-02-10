/**
 * @file motor.h
 * @brief Base motor interface - all protocols implement this
 *
 * This provides a unified API for different motor types (Feetech, Dynamixel,
 * DDSM210, etc). Each protocol registers motors that conform to this interface.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* ──────────────────────────────────────────────
 *  Motor operating modes
 * ────────────────────────────────────────────── */
typedef enum {
    MOTOR_MODE_POSITION,      /* Position control (servo mode) */
    MOTOR_MODE_VELOCITY,      /* Velocity control (wheel mode) */
    MOTOR_MODE_TORQUE,        /* Torque/current control */
    MOTOR_MODE_DISABLED,      /* Motor disabled / free-wheeling */
} motor_mode_t;

/* ──────────────────────────────────────────────
 *  Motor state (read from hardware)
 * ────────────────────────────────────────────── */
typedef struct {
    float position;           /* radians */
    float velocity;           /* rad/s */
    float torque;             /* Nm (or current in A, protocol-dependent) */
    float temperature;        /* Celsius */
    float voltage;            /* Volts */
    uint8_t error_flags;      /* Protocol-specific error bits */
    bool online;              /* Communication OK */
} motor_state_t;

/* ──────────────────────────────────────────────
 *  Motor command (write to hardware)
 * ────────────────────────────────────────────── */
typedef struct {
    motor_mode_t mode;
    float position;           /* Target position (rad) - for POSITION mode */
    float velocity;           /* Target velocity (rad/s) - for VELOCITY mode */
    float torque;             /* Target torque (Nm) - for TORQUE mode */
    float velocity_limit;     /* Max velocity (rad/s) - for POSITION mode */
    float torque_limit;       /* Max torque (Nm) */
} motor_cmd_t;

/* ──────────────────────────────────────────────
 *  Motor interface (vtable)
 * ────────────────────────────────────────────── */
struct motor_s;
typedef struct motor_s motor_t;

typedef struct {
    /* Initialize motor (ping, read model, configure) */
    bool (*init)(motor_t *motor);

    /* Enable/disable torque */
    bool (*set_enabled)(motor_t *motor, bool enabled);

    /* Send command to motor */
    bool (*set_command)(motor_t *motor, const motor_cmd_t *cmd);

    /* Read current state from motor */
    bool (*get_state)(motor_t *motor, motor_state_t *state);

    /* Poll for async responses (call from main loop) */
    void (*poll)(motor_t *motor);
} motor_ops_t;

/* ──────────────────────────────────────────────
 *  Motor instance
 * ────────────────────────────────────────────── */
struct motor_s {
    const motor_ops_t *ops;   /* Protocol-specific operations */
    void *transport;          /* Transport handle (uart, rs485, etc) */
    void *protocol_data;      /* Protocol-specific data */
    uint8_t id;               /* Bus ID */
    const char *name;         /* Joint name (from config) */
    motor_state_t state;      /* Cached state */
    motor_cmd_t cmd;          /* Current command */
    bool enabled;             /* Torque enabled */
};

/* ──────────────────────────────────────────────
 *  Motor registry (generated from config)
 * ────────────────────────────────────────────── */

/**
 * @brief Get motor by index
 * @param index Motor index (0 to motor_count-1)
 * @return Pointer to motor, or NULL if out of range
 */
motor_t *motor_get(uint8_t index);

/**
 * @brief Get motor by name
 * @param name Joint name
 * @return Pointer to motor, or NULL if not found
 */
motor_t *motor_find(const char *name);

/**
 * @brief Get total motor count
 */
uint8_t motor_count(void);

/**
 * @brief Initialize all motors
 */
void motor_init_all(void);

/**
 * @brief Poll all motors (call from main loop)
 */
void motor_poll_all(void);

#endif /* MOTOR_H */