/**
 * @file feetech.c
 * @brief Feetech SCS/STS servo protocol implementation
 */

#include "feetech.h"
#include "pico/stdlib.h"
#include <string.h>

/* ──────────────────────────────────────────────
 *  Packet building
 * ────────────────────────────────────────────── */
static uint8_t feetech_checksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return ~sum;
}

static size_t feetech_build_packet(uint8_t *buf, uint8_t id, uint8_t inst,
                                    const uint8_t *params, size_t param_len) {
    buf[0] = FEETECH_HEADER;
    buf[1] = FEETECH_HEADER;
    buf[2] = id;
    buf[3] = (uint8_t)(param_len + 2);  /* Length = params + inst + checksum */
    buf[4] = inst;

    if (params && param_len > 0) {
        memcpy(&buf[5], params, param_len);
    }

    buf[5 + param_len] = feetech_checksum(&buf[2], param_len + 3);

    return 6 + param_len;
}

static bool feetech_send(motor_t *motor, uint8_t inst,
                         const uint8_t *params, size_t param_len) {
    uint8_t buf[64];
    size_t len = feetech_build_packet(buf, motor->id, inst, params, param_len);
    return transport_write(motor->transport, buf, len);
}

/* ──────────────────────────────────────────────
 *  Packet receiving
 * ────────────────────────────────────────────── */
static bool feetech_receive(motor_t *motor, uint8_t *buf, size_t *len,
                            uint32_t timeout_us) {
    transport_t *t = motor->transport;
    uint32_t start = time_us_32();
    size_t idx = 0;
    size_t expected_len = 0;

    while ((time_us_32() - start) < timeout_us) {
        if (!transport_readable(t)) {
            continue;
        }

        uint8_t byte;
        if (transport_read(t, &byte, 1) != 1) {
            continue;
        }

        /* Wait for header */
        if (idx == 0 && byte != FEETECH_HEADER) continue;
        if (idx == 1 && byte != FEETECH_HEADER) { idx = 0; continue; }

        buf[idx++] = byte;

        /* Got length byte */
        if (idx == 4) {
            expected_len = buf[3] + 4;  /* Header(2) + ID(1) + Len(1) + data */
        }

        /* Complete packet */
        if (expected_len > 0 && idx >= expected_len) {
            /* Verify checksum */
            uint8_t cs = feetech_checksum(&buf[2], expected_len - 3);
            if (cs == buf[expected_len - 1]) {
                *len = expected_len;
                return true;
            }
            /* Bad checksum, reset */
            idx = 0;
            expected_len = 0;
        }

        if (idx >= 64) {
            idx = 0;
            expected_len = 0;
        }
    }

    return false;
}

/* ──────────────────────────────────────────────
 *  Register access
 * ────────────────────────────────────────────── */
static bool feetech_write_reg(motor_t *motor, uint8_t reg,
                               const uint8_t *data, size_t len) {
    uint8_t params[32];
    params[0] = reg;
    memcpy(&params[1], data, len);
    return feetech_send(motor, FEETECH_INST_WRITE, params, len + 1);
}

static bool feetech_read_reg(motor_t *motor, uint8_t reg, uint8_t len,
                              uint8_t *data) {
    uint8_t params[2] = {reg, len};
    if (!feetech_send(motor, FEETECH_INST_READ, params, 2)) {
        return false;
    }

    uint8_t buf[64];
    size_t rx_len;
    if (!feetech_receive(motor, buf, &rx_len, 5000)) {
        return false;
    }

    /* Extract data from response */
    if (rx_len >= 6 && buf[4] == 0) {  /* No error */
        memcpy(data, &buf[5], len);
        return true;
    }

    return false;
}

/* ──────────────────────────────────────────────
 *  Motor operations
 * ────────────────────────────────────────────── */
static bool feetech_init(motor_t *motor) {
    feetech_data_t *data = (feetech_data_t *)motor->protocol_data;

    /* Ping servo */
    if (!feetech_send(motor, FEETECH_INST_PING, NULL, 0)) {
        return false;
    }

    uint8_t buf[64];
    size_t len;
    if (!feetech_receive(motor, buf, &len, 10000)) {
        motor->state.online = false;
        return false;
    }

    motor->state.online = true;

    /* Set default scales for STS3215 */
    data->pos_scale = 4096.0f / (2.0f * 3.14159f);  /* 4096 units per revolution */
    data->vel_scale = 50.0f;   /* ~0.0195 rad/s per unit */
    data->torque_scale = 1.0f;
    data->pos_offset = 2048;   /* Center position */

    return true;
}

static bool feetech_set_enabled(motor_t *motor, bool enabled) {
    uint8_t val = enabled ? 1 : 0;
    if (feetech_write_reg(motor, FEETECH_REG_TORQUE_ENABLE, &val, 1)) {
        motor->enabled = enabled;
        return true;
    }
    return false;
}

static bool feetech_set_command(motor_t *motor, const motor_cmd_t *cmd) {
    feetech_data_t *data = (feetech_data_t *)motor->protocol_data;

    motor->cmd = *cmd;

    if (cmd->mode == MOTOR_MODE_POSITION) {
        /* Convert radians to raw position */
        int32_t raw_pos = (int32_t)(cmd->position * data->pos_scale) + data->pos_offset;
        if (raw_pos < 0) raw_pos = 0;
        if (raw_pos > 4095) raw_pos = 4095;

        uint8_t params[2] = {
            (uint8_t)(raw_pos & 0xFF),
            (uint8_t)((raw_pos >> 8) & 0xFF)
        };
        return feetech_write_reg(motor, FEETECH_REG_GOAL_POS_L, params, 2);

    } else if (cmd->mode == MOTOR_MODE_VELOCITY) {
        /* TODO: Set wheel mode and speed */
        return false;
    }

    return false;
}

static bool feetech_get_state(motor_t *motor, motor_state_t *state) {
    feetech_data_t *data = (feetech_data_t *)motor->protocol_data;

    /* Read position, speed, load, voltage, temp (7 bytes starting at reg 56) */
    uint8_t buf[8];
    if (!feetech_read_reg(motor, FEETECH_REG_PRESENT_POS_L, 8, buf)) {
        state->online = false;
        return false;
    }

    uint16_t raw_pos = buf[0] | (buf[1] << 8);
    int16_t raw_speed = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t raw_load = (int16_t)(buf[4] | (buf[5] << 8));
    uint8_t voltage = buf[6];
    uint8_t temp = buf[7];

    state->position = ((float)raw_pos - data->pos_offset) / data->pos_scale;
    state->velocity = (float)raw_speed / data->vel_scale;
    state->torque = (float)raw_load / data->torque_scale;
    state->voltage = (float)voltage / 10.0f;
    state->temperature = (float)temp;
    state->online = true;
    state->error_flags = 0;

    motor->state = *state;
    return true;
}

static void feetech_poll(motor_t *motor) {
    feetech_data_t *data = (feetech_data_t *)motor->protocol_data;
    uint32_t now = time_us_32();

    /* Poll every 10ms */
    if ((now - data->last_poll_us) >= 10000) {
        data->last_poll_us = now;
        feetech_get_state(motor, &motor->state);
    }
}

/* ──────────────────────────────────────────────
 *  Public API
 * ────────────────────────────────────────────── */
static const motor_ops_t feetech_ops = {
    .init = feetech_init,
    .set_enabled = feetech_set_enabled,
    .set_command = feetech_set_command,
    .get_state = feetech_get_state,
    .poll = feetech_poll,
};

const motor_ops_t *feetech_get_ops(void) {
    return &feetech_ops;
}

void feetech_motor_init(motor_t *motor, transport_t *transport,
                        uint8_t id, const char *name) {
    static feetech_data_t data_instances[16];
    static uint8_t data_count = 0;

    if (data_count >= 16) return;

    memset(motor, 0, sizeof(*motor));
    motor->ops = &feetech_ops;
    motor->transport = transport;
    motor->protocol_data = &data_instances[data_count++];
    motor->id = id;
    motor->name = name;
    motor->enabled = false;

    memset(motor->protocol_data, 0, sizeof(feetech_data_t));
}