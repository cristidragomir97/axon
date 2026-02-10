/**
 * @file ddsm210.c
 * @brief Waveshare DDSM210 brushless motor protocol implementation
 *
 * Protocol: 115200 baud, 10-byte packets, CRC-8/MAXIM checksum
 * Reference: https://www.waveshare.com/wiki/DDSM210
 */

#include "ddsm210.h"
#include "pico/stdlib.h"
#include <string.h>

/* ──────────────────────────────────────────────
 *  CRC-8/MAXIM calculation
 *  Polynomial: x^8 + x^5 + x^4 + 1 (0x31)
 *  Init: 0x00, RefIn: true, RefOut: true, XorOut: 0x00
 * ────────────────────────────────────────────── */
static const uint8_t crc8_table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

static uint8_t ddsm210_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* ──────────────────────────────────────────────
 *  Packet handling
 * ────────────────────────────────────────────── */
static void ddsm210_build_packet(uint8_t *buf, uint8_t id, uint8_t cmd,
                                  const uint8_t *data, size_t data_len) {
    buf[0] = id;
    buf[1] = cmd;

    /* Fill data bytes (max 7) */
    for (size_t i = 0; i < 7; i++) {
        buf[2 + i] = (i < data_len) ? data[i] : 0x00;
    }

    /* CRC over bytes 0-8 */
    buf[9] = ddsm210_crc8(buf, 9);
}

static bool ddsm210_send(motor_t *motor, uint8_t cmd,
                          const uint8_t *data, size_t data_len) {
    uint8_t buf[DDSM210_PACKET_LEN];
    ddsm210_build_packet(buf, motor->id, cmd, data, data_len);
    return transport_write(motor->transport, buf, DDSM210_PACKET_LEN);
}

static bool ddsm210_receive(motor_t *motor, uint8_t *buf, uint32_t timeout_us) {
    transport_t *t = motor->transport;
    uint32_t start = time_us_32();
    size_t idx = 0;

    while ((time_us_32() - start) < timeout_us) {
        if (!transport_readable(t)) {
            continue;
        }

        uint8_t byte;
        if (transport_read(t, &byte, 1) != 1) {
            continue;
        }

        /* First byte should be our ID */
        if (idx == 0 && byte != motor->id) {
            continue;
        }

        buf[idx++] = byte;

        /* Complete packet */
        if (idx >= DDSM210_PACKET_LEN) {
            /* Verify CRC */
            uint8_t crc = ddsm210_crc8(buf, 9);
            if (crc == buf[9]) {
                return true;
            }
            /* Bad CRC, reset */
            idx = 0;
        }
    }

    return false;
}

/* ──────────────────────────────────────────────
 *  Motor operations
 * ────────────────────────────────────────────── */
static bool ddsm210_init(motor_t *motor) {
    ddsm210_data_t *data = (ddsm210_data_t *)motor->protocol_data;

    /* Query current mode to verify communication */
    uint8_t query_data[7] = {0};
    if (!ddsm210_send(motor, DDSM210_CMD_QUERY_MODE, query_data, 7)) {
        return false;
    }

    uint8_t buf[DDSM210_PACKET_LEN];
    if (!ddsm210_receive(motor, buf, 10000)) {
        motor->state.online = false;
        return false;
    }

    data->mode = buf[2];
    motor->state.online = true;

    /* Set to velocity mode by default for diff drive */
    if (data->mode != DDSM210_MODE_VELOCITY) {
        ddsm210_set_mode(motor, DDSM210_MODE_VELOCITY);
    }

    return true;
}

static bool ddsm210_set_enabled(motor_t *motor, bool enabled) {
    /* DDSM210 doesn't have explicit enable/disable - just stop motor */
    if (!enabled) {
        motor_cmd_t cmd = {.mode = MOTOR_MODE_VELOCITY, .velocity = 0.0f};
        motor->ops->set_command(motor, &cmd);
    }
    motor->enabled = enabled;
    return true;
}

static bool ddsm210_set_command(motor_t *motor, const motor_cmd_t *cmd) {
    ddsm210_data_t *data = (ddsm210_data_t *)motor->protocol_data;

    motor->cmd = *cmd;

    if (cmd->mode == MOTOR_MODE_VELOCITY) {
        /* Ensure we're in velocity mode */
        if (data->mode != DDSM210_MODE_VELOCITY) {
            ddsm210_set_mode(motor, DDSM210_MODE_VELOCITY);
        }

        /* Convert rad/s to 0.1 RPM units */
        float rpm = cmd->velocity * DDSM210_RAD_TO_RPM;
        int16_t raw_speed = (int16_t)(rpm * DDSM210_RPM_SCALE);

        /* Clamp to valid range */
        if (raw_speed > 2100) raw_speed = 2100;
        if (raw_speed < -2100) raw_speed = -2100;

        /* Build drive command */
        /* DATA: speed(2), fb1(1), fb2(1), accel_time(1), brake(1), reserved(1) */
        uint8_t drive_data[7] = {
            (uint8_t)(raw_speed & 0xFF),
            (uint8_t)((raw_speed >> 8) & 0xFF),
            DDSM210_FB_VELOCITY,    /* Feedback 1: velocity */
            DDSM210_FB_POSITION,    /* Feedback 2: position */
            0x00,                   /* Acceleration time */
            0x00,                   /* Brake flag */
            0x00                    /* Reserved */
        };

        if (!ddsm210_send(motor, DDSM210_CMD_DRIVE, drive_data, 7)) {
            return false;
        }

        /* Parse response */
        uint8_t buf[DDSM210_PACKET_LEN];
        if (ddsm210_receive(motor, buf, 5000)) {
            /* Response: ID, CMD, fb1(2), fb2(2), accel_time, temp, error, CRC */
            data->raw_velocity = (int16_t)(buf[2] | (buf[3] << 8));
            data->raw_position = (uint16_t)(buf[4] | (buf[5] << 8));
            data->temperature = buf[7];
            data->error_code = buf[8];
            data->last_cmd_us = time_us_32();

            /* Update state */
            motor->state.velocity = (float)data->raw_velocity / DDSM210_RPM_SCALE * DDSM210_RPM_TO_RAD;
            motor->state.position = (float)data->raw_position / DDSM210_POS_SCALE * (3.14159265f / 180.0f);
            motor->state.temperature = (float)data->temperature;
            motor->state.error_flags = data->error_code;
            motor->state.online = true;
        }

        return true;

    } else if (cmd->mode == MOTOR_MODE_POSITION) {
        /* Ensure we're in position mode */
        if (data->mode != DDSM210_MODE_POSITION) {
            ddsm210_set_mode(motor, DDSM210_MODE_POSITION);
        }

        /* Convert radians to raw position (0-32767 for 0-360 degrees) */
        float degrees = cmd->position * (180.0f / 3.14159265f);
        while (degrees < 0) degrees += 360.0f;
        while (degrees >= 360.0f) degrees -= 360.0f;

        uint16_t raw_pos = (uint16_t)(degrees * DDSM210_POS_SCALE);
        if (raw_pos > 32767) raw_pos = 32767;

        /* Build drive command for position mode */
        uint8_t drive_data[7] = {
            (uint8_t)(raw_pos & 0xFF),
            (uint8_t)((raw_pos >> 8) & 0xFF),
            DDSM210_FB_VELOCITY,
            DDSM210_FB_POSITION,
            0x00,
            0x00,
            0x00
        };

        if (!ddsm210_send(motor, DDSM210_CMD_DRIVE, drive_data, 7)) {
            return false;
        }

        uint8_t buf[DDSM210_PACKET_LEN];
        if (ddsm210_receive(motor, buf, 5000)) {
            data->raw_velocity = (int16_t)(buf[2] | (buf[3] << 8));
            data->raw_position = (uint16_t)(buf[4] | (buf[5] << 8));
            data->temperature = buf[7];
            data->error_code = buf[8];
            data->last_cmd_us = time_us_32();

            motor->state.velocity = (float)data->raw_velocity / DDSM210_RPM_SCALE * DDSM210_RPM_TO_RAD;
            motor->state.position = (float)data->raw_position / DDSM210_POS_SCALE * (3.14159265f / 180.0f);
            motor->state.temperature = (float)data->temperature;
            motor->state.error_flags = data->error_code;
            motor->state.online = true;
        }

        return true;
    }

    return false;
}

static bool ddsm210_get_state(motor_t *motor, motor_state_t *state) {
    ddsm210_data_t *data = (ddsm210_data_t *)motor->protocol_data;

    /* Query mileage and position */
    uint8_t query_data[7] = {0};
    if (!ddsm210_send(motor, DDSM210_CMD_QUERY_POS, query_data, 7)) {
        state->online = false;
        return false;
    }

    uint8_t buf[DDSM210_PACKET_LEN];
    if (!ddsm210_receive(motor, buf, 5000)) {
        state->online = false;
        return false;
    }

    /* Response: ID, CMD, mileage(4), position(2), error, CRC */
    data->mileage_laps = (int32_t)(buf[2] | (buf[3] << 8) | (buf[4] << 16) | (buf[5] << 24));
    data->raw_position = (uint16_t)(buf[6] | (buf[7] << 8));
    data->error_code = buf[8];

    /* Convert to engineering units */
    state->position = (float)data->raw_position / DDSM210_POS_SCALE * (3.14159265f / 180.0f);
    state->velocity = motor->state.velocity;  /* Keep last known velocity */
    state->torque = 0.0f;  /* Not available from query */
    state->temperature = (float)data->temperature;
    state->error_flags = data->error_code;
    state->online = true;

    motor->state = *state;
    return true;
}

static void ddsm210_poll(motor_t *motor) {
    ddsm210_data_t *data = (ddsm210_data_t *)motor->protocol_data;
    uint32_t now = time_us_32();

    /* If we haven't sent a command recently, send a zero-velocity
       command to get feedback (motor expects continuous commands) */
    if ((now - data->last_cmd_us) >= 50000) {  /* 50ms timeout */
        if (motor->cmd.mode == MOTOR_MODE_VELOCITY) {
            motor->ops->set_command(motor, &motor->cmd);
        }
    }

    /* Also do periodic state query every 100ms */
    if ((now - data->last_poll_us) >= 100000) {
        data->last_poll_us = now;
        /* Query is done via set_command feedback, no need for separate query */
    }
}

/* ──────────────────────────────────────────────
 *  Public API
 * ────────────────────────────────────────────── */
bool ddsm210_set_mode(motor_t *motor, uint8_t mode) {
    ddsm210_data_t *data = (ddsm210_data_t *)motor->protocol_data;

    uint8_t mode_data[7] = {mode, 0, 0, 0, 0, 0, 0};
    if (!ddsm210_send(motor, DDSM210_CMD_SET_MODE, mode_data, 7)) {
        return false;
    }

    /* Mode change doesn't return a response, just wait a bit */
    sleep_ms(10);

    data->mode = mode;
    return true;
}

bool ddsm210_query_position(motor_t *motor, int32_t *laps, uint16_t *position) {
    motor_state_t state;
    if (!ddsm210_get_state(motor, &state)) {
        return false;
    }

    ddsm210_data_t *data = (ddsm210_data_t *)motor->protocol_data;
    if (laps) *laps = data->mileage_laps;
    if (position) *position = data->raw_position;

    return true;
}

static const motor_ops_t ddsm210_ops = {
    .init = ddsm210_init,
    .set_enabled = ddsm210_set_enabled,
    .set_command = ddsm210_set_command,
    .get_state = ddsm210_get_state,
    .poll = ddsm210_poll,
};

const motor_ops_t *ddsm210_get_ops(void) {
    return &ddsm210_ops;
}

void ddsm210_motor_init(motor_t *motor, transport_t *transport,
                        uint8_t id, const char *name) {
    static ddsm210_data_t data_instances[8];
    static uint8_t data_count = 0;

    if (data_count >= 8) return;

    memset(motor, 0, sizeof(*motor));
    motor->ops = &ddsm210_ops;
    motor->transport = transport;
    motor->protocol_data = &data_instances[data_count++];
    motor->id = id;
    motor->name = name;
    motor->enabled = false;

    memset(motor->protocol_data, 0, sizeof(ddsm210_data_t));
}
