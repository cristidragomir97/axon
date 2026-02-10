/**
 * @file main.c
 * @brief RoboCore Axon Sketch — Diff Drive + 6DOF Arm + Pan/Tilt + IMU
 *
 * ROS Topics:
 *   Subscribes:
 *     - base_cmd    (std_msgs/Float64MultiArray) - [left, right] rad/s
 *     - arm_cmd     (std_msgs/Float64MultiArray) - [j1..j6] rad
 *     - camera_cmd  (std_msgs/Float64MultiArray) - [pan, tilt] rad
 *
 *   Publishes:
 *     - joint_states (sensor_msgs/JointState) - position/velocity for all joints
 *     - imu/data     (sensor_msgs/Imu) - orientation, gyro, accel
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tusb.h"

#include "config.h"
#include "pinout.h"
#include "board.h"

#include "robot_config.h"
#include "motors.h"
#include "ros.h"

/* BNO055 library */
#include "bno055.h"

#ifdef ENABLE_LIDAR_BRIDGE
#include "lidar_bridge.h"
#endif

/* ──────────────────────────────────────────────
 *  IMU
 * ────────────────────────────────────────────── */
static bno055_t imu;

/* ──────────────────────────────────────────────
 *  Command buffers
 * ────────────────────────────────────────────── */
static float base_cmd[NUM_WHEELS] = {0};
static float arm_cmd[NUM_ARM] = {0};
static float camera_cmd[NUM_CAMERA] = {0};

/* ──────────────────────────────────────────────
 *  Control loop
 * ────────────────────────────────────────────── */
static void control_loop(void) {
    /* Check for new ROS commands */
    if (ros_get_base_cmd(base_cmd)) {
        motors_set_wheel_velocities(base_cmd);
    }
    if (ros_get_arm_cmd(arm_cmd)) {
        motors_set_arm_positions(arm_cmd);
    }
    if (ros_get_camera_cmd(camera_cmd)) {
        motors_set_camera_positions(camera_cmd);
    }

    /* Poll motors for feedback */
    motors_poll();
}

/* ──────────────────────────────────────────────
 *  Publishing
 * ────────────────────────────────────────────── */
static void publish_joint_states(void) {
    float positions[NUM_MOTORS];
    float velocities[NUM_MOTORS];
    int idx = 0;

    /* Wheels */
    for (int i = 0; i < NUM_WHEELS; i++) {
        positions[idx] = wheels[i].state.position;
        velocities[idx] = wheels[i].state.velocity;
        idx++;
    }

    /* Arm */
    for (int i = 0; i < NUM_ARM; i++) {
        positions[idx] = arm[i].state.position;
        velocities[idx] = arm[i].state.velocity;
        idx++;
    }

    /* Camera */
    for (int i = 0; i < NUM_CAMERA; i++) {
        positions[idx] = camera[i].state.position;
        velocities[idx] = camera[i].state.velocity;
        idx++;
    }

    ros_publish_joint_states(positions, velocities);
}

static void publish_imu_data(void) {
    bno055_vec3_t accel, gyro;
    bno055_quaternion_t quat;

    bno055_get_accel(&imu, &accel);
    bno055_get_gyro(&imu, &gyro);
    bno055_get_quaternion(&imu, &quat);

    float orientation[4] = {quat.w, quat.x, quat.y, quat.z};
    float angular_velocity[3] = {gyro.x, gyro.y, gyro.z};
    float linear_acceleration[3] = {accel.x, accel.y, accel.z};

    ros_publish_imu(orientation, angular_velocity, linear_acceleration);
}

/* ──────────────────────────────────────────────
 *  Core 1: USB
 * ────────────────────────────────────────────── */
static void core1_entry(void) {
    tusb_init();

#ifdef ENABLE_LIDAR_BRIDGE
    lidar_bridge_init();
#endif

    while (1) {
        tud_task();

#ifdef ENABLE_LIDAR_BRIDGE
        lidar_bridge_task();
#endif
    }
}

/* ──────────────────────────────────────────────
 *  Main
 * ────────────────────────────────────────────── */
int main(void) {
    /* Hardware init */
    board_init();

    /* Motors */
    motors_init();
    motors_set_enabled(true);

    /* IMU */
    bno055_init(&imu, SENSOR_I2C, I2C_ADDR_BNO055);
    bno055_set_mode(&imu, BNO055_MODE_NDOF);

    /* USB on Core 1 */
    multicore_launch_core1(core1_entry);

    /* ROS init */
    ros_init();

    /* Main loop */
    uint32_t last_control = time_us_32();
    uint32_t last_publish = time_us_32();

    while (1) {
        uint32_t now = time_us_32();

        /* Control at 100Hz */
        if ((now - last_control) >= CONTROL_PERIOD_US) {
            last_control = now;
            control_loop();
        }

        /* Publish at 50Hz */
        if ((now - last_publish) >= PUBLISH_PERIOD_US) {
            last_publish = now;
            publish_joint_states();
            publish_imu_data();
        }
    }
}

/* ──────────────────────────────────────────────
 *  TinyUSB callbacks
 * ────────────────────────────────────────────── */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf; (void)dtr; (void)rts;
}

void tud_cdc_rx_cb(uint8_t itf) {
#ifdef ENABLE_LIDAR_BRIDGE
    if (itf == USB_CDC_LIDAR) {
        lidar_bridge_host_rx();
    }
#else
    (void)itf;
#endif
}
