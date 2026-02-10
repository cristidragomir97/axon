/**
 * @file motors.h
 * @brief Motor instances and control
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "motor.h"
#include "robot_config.h"

/* Motor arrays */
extern motor_t wheels[NUM_WHEELS];
extern motor_t arm[NUM_ARM];
extern motor_t camera[NUM_CAMERA];

/* Joint names for ROS */
extern const char *wheel_names[NUM_WHEELS];
extern const char *arm_names[NUM_ARM];
extern const char *camera_names[NUM_CAMERA];

/**
 * @brief Initialize all motors
 */
void motors_init(void);

/**
 * @brief Poll all motors for state updates
 */
void motors_poll(void);

/**
 * @brief Enable/disable all motor torque
 */
void motors_set_enabled(bool enabled);

/**
 * @brief Send velocity commands to wheels
 * @param velocities Array of [left, right] in rad/s
 */
void motors_set_wheel_velocities(const float *velocities);

/**
 * @brief Send position commands to arm
 * @param positions Array of [j1..j6] in rad
 */
void motors_set_arm_positions(const float *positions);

/**
 * @brief Send position commands to camera
 * @param positions Array of [pan, tilt] in rad
 */
void motors_set_camera_positions(const float *positions);

#endif /* MOTORS_H */
