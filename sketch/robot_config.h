/**
 * @file robot_config.h
 * @brief Robot configuration - motor IDs, counts, names
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ──────────────────────────────────────────────
 *  Wheel motors (DDSM210 on bus_ddsm)
 * ────────────────────────────────────────────── */
#define WHEEL_LEFT_ID   1
#define WHEEL_RIGHT_ID  2
#define NUM_WHEELS      2

/* ──────────────────────────────────────────────
 *  Arm servos (Feetech on bus_feetech)
 * ────────────────────────────────────────────── */
#define ARM_J1_ID  1   /* shoulder_pan */
#define ARM_J2_ID  2   /* shoulder_lift */
#define ARM_J3_ID  3   /* elbow */
#define ARM_J4_ID  4   /* wrist_pitch */
#define ARM_J5_ID  5   /* wrist_roll */
#define ARM_J6_ID  6   /* gripper */
#define NUM_ARM    6

/* ──────────────────────────────────────────────
 *  Camera servos (Feetech on bus_feetech)
 * ────────────────────────────────────────────── */
#define CAM_PAN_ID   7
#define CAM_TILT_ID  8
#define NUM_CAMERA   2

/* ──────────────────────────────────────────────
 *  Totals
 * ────────────────────────────────────────────── */
#define NUM_MOTORS  (NUM_WHEELS + NUM_ARM + NUM_CAMERA)

/* ──────────────────────────────────────────────
 *  Timing
 * ────────────────────────────────────────────── */
#define CONTROL_RATE_HZ     100
#define CONTROL_PERIOD_US   (1000000 / CONTROL_RATE_HZ)

#define PUBLISH_RATE_HZ     50
#define PUBLISH_PERIOD_US   (1000000 / PUBLISH_RATE_HZ)

#endif /* ROBOT_CONFIG_H */
