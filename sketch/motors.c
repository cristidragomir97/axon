/**
 * @file motors.c
 * @brief Motor instances and control implementation
 */

#include "motors.h"
#include "board.h"
#include "protocol/feetech.h"
#include "protocol/ddsm210.h"

#include <string.h>

/* ──────────────────────────────────────────────
 *  Motor instances
 * ────────────────────────────────────────────── */
motor_t wheels[NUM_WHEELS];
motor_t arm[NUM_ARM];
motor_t camera[NUM_CAMERA];

/* Joint names */
const char *wheel_names[NUM_WHEELS] = {"wheel_left", "wheel_right"};
const char *arm_names[NUM_ARM] = {
    "shoulder_pan", "shoulder_lift", "elbow",
    "wrist_pitch", "wrist_roll", "gripper"
};
const char *camera_names[NUM_CAMERA] = {"camera_pan", "camera_tilt"};

/* ──────────────────────────────────────────────
 *  Initialization
 * ────────────────────────────────────────────── */
void motors_init(void) {
    /* Wheels - DDSM210 on bus_ddsm (velocity mode) */
    ddsm210_motor_init(&wheels[0], &bus_ddsm, WHEEL_LEFT_ID, wheel_names[0]);
    ddsm210_motor_init(&wheels[1], &bus_ddsm, WHEEL_RIGHT_ID, wheel_names[1]);

    /* Arm - Feetech on bus_feetech */
    feetech_motor_init(&arm[0], &bus_feetech, ARM_J1_ID, arm_names[0]);
    feetech_motor_init(&arm[1], &bus_feetech, ARM_J2_ID, arm_names[1]);
    feetech_motor_init(&arm[2], &bus_feetech, ARM_J3_ID, arm_names[2]);
    feetech_motor_init(&arm[3], &bus_feetech, ARM_J4_ID, arm_names[3]);
    feetech_motor_init(&arm[4], &bus_feetech, ARM_J5_ID, arm_names[4]);
    feetech_motor_init(&arm[5], &bus_feetech, ARM_J6_ID, arm_names[5]);

    /* Camera - Feetech on bus_feetech */
    feetech_motor_init(&camera[0], &bus_feetech, CAM_PAN_ID, camera_names[0]);
    feetech_motor_init(&camera[1], &bus_feetech, CAM_TILT_ID, camera_names[1]);

    /* Initialize hardware */
    for (int i = 0; i < NUM_WHEELS; i++) {
        if (wheels[i].ops && wheels[i].ops->init) {
            wheels[i].ops->init(&wheels[i]);
        }
    }
    for (int i = 0; i < NUM_ARM; i++) {
        if (arm[i].ops && arm[i].ops->init) {
            arm[i].ops->init(&arm[i]);
        }
    }
    for (int i = 0; i < NUM_CAMERA; i++) {
        if (camera[i].ops && camera[i].ops->init) {
            camera[i].ops->init(&camera[i]);
        }
    }
}

/* ──────────────────────────────────────────────
 *  Polling
 * ────────────────────────────────────────────── */
void motors_poll(void) {
    for (int i = 0; i < NUM_WHEELS; i++) {
        if (wheels[i].ops && wheels[i].ops->poll) {
            wheels[i].ops->poll(&wheels[i]);
        }
    }
    for (int i = 0; i < NUM_ARM; i++) {
        if (arm[i].ops && arm[i].ops->poll) {
            arm[i].ops->poll(&arm[i]);
        }
    }
    for (int i = 0; i < NUM_CAMERA; i++) {
        if (camera[i].ops && camera[i].ops->poll) {
            camera[i].ops->poll(&camera[i]);
        }
    }
}

/* ──────────────────────────────────────────────
 *  Enable/Disable
 * ────────────────────────────────────────────── */
void motors_set_enabled(bool enabled) {
    for (int i = 0; i < NUM_WHEELS; i++) {
        if (wheels[i].ops && wheels[i].ops->set_enabled) {
            wheels[i].ops->set_enabled(&wheels[i], enabled);
        }
    }
    for (int i = 0; i < NUM_ARM; i++) {
        if (arm[i].ops && arm[i].ops->set_enabled) {
            arm[i].ops->set_enabled(&arm[i], enabled);
        }
    }
    for (int i = 0; i < NUM_CAMERA; i++) {
        if (camera[i].ops && camera[i].ops->set_enabled) {
            camera[i].ops->set_enabled(&camera[i], enabled);
        }
    }
}

/* ──────────────────────────────────────────────
 *  Commands
 * ────────────────────────────────────────────── */
void motors_set_wheel_velocities(const float *velocities) {
    motor_cmd_t cmd = {.mode = MOTOR_MODE_VELOCITY};

    for (int i = 0; i < NUM_WHEELS; i++) {
        if (wheels[i].ops && wheels[i].ops->set_command) {
            cmd.velocity = velocities[i];
            wheels[i].ops->set_command(&wheels[i], &cmd);
        }
    }
}

void motors_set_arm_positions(const float *positions) {
    motor_cmd_t cmd = {.mode = MOTOR_MODE_POSITION};

    for (int i = 0; i < NUM_ARM; i++) {
        if (arm[i].ops && arm[i].ops->set_command) {
            cmd.position = positions[i];
            arm[i].ops->set_command(&arm[i], &cmd);
        }
    }
}

void motors_set_camera_positions(const float *positions) {
    motor_cmd_t cmd = {.mode = MOTOR_MODE_POSITION};

    for (int i = 0; i < NUM_CAMERA; i++) {
        if (camera[i].ops && camera[i].ops->set_command) {
            cmd.position = positions[i];
            camera[i].ops->set_command(&camera[i], &cmd);
        }
    }
}
