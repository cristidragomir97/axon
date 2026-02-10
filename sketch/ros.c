/**
 * @file ros.c
 * @brief ROS 2 integration via picoros
 */

#include "ros.h"
#include "motors.h"
#include "pico/time.h"

#include "picoros.h"
#include "picoserdes.h"
#include "example_types.h"

#include <string.h>

/* ──────────────────────────────────────────────
 *  Picoros interface and node
 * ────────────────────────────────────────────── */
static picoros_interface_t ros_interface = {
    .mode = "client",
    .locator = "",  /* Uses USB CDC transport */
};

static picoros_node_t ros_node = {
    .name = "robocore_axon",
    .domain_id = 0,
};

/* ──────────────────────────────────────────────
 *  Publishers
 * ────────────────────────────────────────────── */
static picoros_publisher_t pub_joint_states;
static picoros_publisher_t pub_imu;

static uint8_t pub_buf[1024];

/* ──────────────────────────────────────────────
 *  Subscribers
 * ────────────────────────────────────────────── */
static picoros_subscriber_t sub_base_cmd;
static picoros_subscriber_t sub_arm_cmd;
static picoros_subscriber_t sub_camera_cmd;

/* Command buffers (double-buffered for thread safety) */
static float cmd_base[NUM_WHEELS];
static float cmd_arm[NUM_ARM];
static float cmd_camera[NUM_CAMERA];

static volatile bool cmd_base_new = false;
static volatile bool cmd_arm_new = false;
static volatile bool cmd_camera_new = false;

/* ──────────────────────────────────────────────
 *  Subscriber callbacks
 * ────────────────────────────────────────────── */
static void on_base_cmd_cb(uint8_t *rx_data, size_t data_len) {
    ros_Float64MultiArray msg;
    static double data_buf[NUM_WHEELS];
    msg.data.data = data_buf;
    msg.data.n_elements = 0;

    if (ps_deserialize(rx_data, &msg, data_len)) {
        size_t n = msg.data.n_elements < NUM_WHEELS ? msg.data.n_elements : NUM_WHEELS;
        for (size_t i = 0; i < n; i++) {
            cmd_base[i] = (float)msg.data.data[i];
        }
        cmd_base_new = true;
    }
}

static void on_arm_cmd_cb(uint8_t *rx_data, size_t data_len) {
    ros_Float64MultiArray msg;
    static double data_buf[NUM_ARM];
    msg.data.data = data_buf;
    msg.data.n_elements = 0;

    if (ps_deserialize(rx_data, &msg, data_len)) {
        size_t n = msg.data.n_elements < NUM_ARM ? msg.data.n_elements : NUM_ARM;
        for (size_t i = 0; i < n; i++) {
            cmd_arm[i] = (float)msg.data.data[i];
        }
        cmd_arm_new = true;
    }
}

static void on_camera_cmd_cb(uint8_t *rx_data, size_t data_len) {
    ros_Float64MultiArray msg;
    static double data_buf[NUM_CAMERA];
    msg.data.data = data_buf;
    msg.data.n_elements = 0;

    if (ps_deserialize(rx_data, &msg, data_len)) {
        size_t n = msg.data.n_elements < NUM_CAMERA ? msg.data.n_elements : NUM_CAMERA;
        for (size_t i = 0; i < n; i++) {
            cmd_camera[i] = (float)msg.data.data[i];
        }
        cmd_camera_new = true;
    }
}

/* ──────────────────────────────────────────────
 *  Initialization
 * ────────────────────────────────────────────── */
bool ros_init(void) {
    /* Initialize interface with retries */
    picoros_res_t res;
    for (int i = 0; i < 10; i++) {
        res = picoros_interface_init(&ros_interface);
        if (res == PICOROS_OK) break;
        sleep_ms(100);
    }
    if (res != PICOROS_OK) {
        return false;
    }

    /* Initialize node */
    res = picoros_node_init(&ros_node);
    if (res != PICOROS_OK) {
        return false;
    }

    /* Declare publishers */
    pub_joint_states.topic.name = "joint_states";
    pub_joint_states.topic.type = "sensor_msgs::msg::dds_::JointState";
    res = picoros_publisher_declare(&ros_node, &pub_joint_states);
    if (res != PICOROS_OK) {
        return false;
    }

    pub_imu.topic.name = "imu/data";
    pub_imu.topic.type = "sensor_msgs::msg::dds_::Imu";
    res = picoros_publisher_declare(&ros_node, &pub_imu);
    if (res != PICOROS_OK) {
        return false;
    }

    /* Declare subscribers */
    sub_base_cmd.topic.name = "base_cmd";
    sub_base_cmd.topic.type = "std_msgs::msg::dds_::Float64MultiArray";
    sub_base_cmd.user_callback = on_base_cmd_cb;
    res = picoros_subscriber_declare(&ros_node, &sub_base_cmd);
    if (res != PICOROS_OK) {
        return false;
    }

    sub_arm_cmd.topic.name = "arm_cmd";
    sub_arm_cmd.topic.type = "std_msgs::msg::dds_::Float64MultiArray";
    sub_arm_cmd.user_callback = on_arm_cmd_cb;
    res = picoros_subscriber_declare(&ros_node, &sub_arm_cmd);
    if (res != PICOROS_OK) {
        return false;
    }

    sub_camera_cmd.topic.name = "camera_cmd";
    sub_camera_cmd.topic.type = "std_msgs::msg::dds_::Float64MultiArray";
    sub_camera_cmd.user_callback = on_camera_cmd_cb;
    res = picoros_subscriber_declare(&ros_node, &sub_camera_cmd);
    if (res != PICOROS_OK) {
        return false;
    }

    return true;
}

bool ros_connected(void) {
    /* picoros is event-driven, always "connected" once initialized */
    return true;
}

/* ──────────────────────────────────────────────
 *  Publishing
 * ────────────────────────────────────────────── */
void ros_publish_joint_states(const float *positions, const float *velocities) {
    /* Get current time */
    uint64_t now_us = time_us_64();
    uint32_t sec = (uint32_t)(now_us / 1000000);
    uint32_t nsec = (uint32_t)((now_us % 1000000) * 1000);

    /* Build arrays */
    static double pos_buf[NUM_MOTORS];
    static double vel_buf[NUM_MOTORS];
    static double eff_buf[NUM_MOTORS];

    for (int i = 0; i < NUM_MOTORS; i++) {
        pos_buf[i] = (double)positions[i];
        vel_buf[i] = (double)velocities[i];
        eff_buf[i] = 0.0;
    }

    /* Joint names */
    static char *names[NUM_MOTORS];
    int idx = 0;
    for (int i = 0; i < NUM_WHEELS; i++) {
        names[idx++] = (char *)wheel_names[i];
    }
    for (int i = 0; i < NUM_ARM; i++) {
        names[idx++] = (char *)arm_names[i];
    }
    for (int i = 0; i < NUM_CAMERA; i++) {
        names[idx++] = (char *)camera_names[i];
    }

    /* Build message */
    static char frame_id[] = "";
    ros_JointState msg = {
        .header = {
            .stamp = {
                .sec = (int32_t)sec,
                .nanosec = nsec,
            },
            .frame_id = {.data = frame_id, .n_elements = 0},
        },
        .name = {.data = names, .n_elements = NUM_MOTORS},
        .position = {.data = pos_buf, .n_elements = NUM_MOTORS},
        .velocity = {.data = vel_buf, .n_elements = NUM_MOTORS},
        .effort = {.data = eff_buf, .n_elements = NUM_MOTORS},
    };

    /* Serialize and publish */
    size_t len = ps_serialize(pub_buf, &msg, sizeof(pub_buf));
    if (len > 0) {
        picoros_publish(&pub_joint_states, pub_buf, len);
    }
}

void ros_publish_imu(const float *orientation,
                     const float *angular_velocity,
                     const float *linear_acceleration) {
    /* Get current time */
    uint64_t now_us = time_us_64();
    uint32_t sec = (uint32_t)(now_us / 1000000);
    uint32_t nsec = (uint32_t)((now_us % 1000000) * 1000);

    /* Build message */
    static char frame_id[] = "imu_link";
    ros_Imu msg = {
        .header = {
            .stamp = {
                .sec = (int32_t)sec,
                .nanosec = nsec,
            },
            .frame_id = {.data = frame_id, .n_elements = 8},
        },
        .orientation = {
            .w = (double)orientation[0],
            .x = (double)orientation[1],
            .y = (double)orientation[2],
            .z = (double)orientation[3],
        },
        .orientation_covariance = {0},
        .angular_velocity = {
            .x = (double)angular_velocity[0],
            .y = (double)angular_velocity[1],
            .z = (double)angular_velocity[2],
        },
        .angular_velocity_covariance = {0},
        .linear_acceleration = {
            .x = (double)linear_acceleration[0],
            .y = (double)linear_acceleration[1],
            .z = (double)linear_acceleration[2],
        },
        .linear_acceleration_covariance = {0},
    };

    /* Serialize and publish */
    size_t len = ps_serialize(pub_buf, &msg, sizeof(pub_buf));
    if (len > 0) {
        picoros_publish(&pub_imu, pub_buf, len);
    }
}

/* ──────────────────────────────────────────────
 *  Command retrieval
 * ────────────────────────────────────────────── */
bool ros_get_base_cmd(float *velocities) {
    if (!cmd_base_new) return false;
    cmd_base_new = false;
    memcpy(velocities, cmd_base, NUM_WHEELS * sizeof(float));
    return true;
}

bool ros_get_arm_cmd(float *positions) {
    if (!cmd_arm_new) return false;
    cmd_arm_new = false;
    memcpy(positions, cmd_arm, NUM_ARM * sizeof(float));
    return true;
}

bool ros_get_camera_cmd(float *positions) {
    if (!cmd_camera_new) return false;
    cmd_camera_new = false;
    memcpy(positions, cmd_camera, NUM_CAMERA * sizeof(float));
    return true;
}
