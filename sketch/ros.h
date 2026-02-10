/**
 * @file ros.h
 * @brief ROS 2 integration via picoros
 *
 * Publishers:
 *   - joint_states (sensor_msgs/JointState)
 *   - imu/data (sensor_msgs/Imu)
 *
 * Subscribers:
 *   - base_cmd (std_msgs/Float64MultiArray)
 *   - arm_cmd (std_msgs/Float64MultiArray)
 *   - camera_cmd (std_msgs/Float64MultiArray)
 */

#ifndef ROS_H
#define ROS_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_config.h"

/**
 * @brief Initialize ROS node and declare publishers/subscribers
 * @return true on success
 */
bool ros_init(void);

/**
 * @brief Check if ROS is connected
 */
bool ros_connected(void);

/**
 * @brief Publish joint states for all motors
 * @param positions Array of positions [wheels..., arm..., camera...]
 * @param velocities Array of velocities [wheels..., arm..., camera...]
 */
void ros_publish_joint_states(const float *positions, const float *velocities);

/**
 * @brief Publish IMU data
 * @param orientation Quaternion [w, x, y, z]
 * @param angular_velocity Gyroscope [x, y, z] rad/s
 * @param linear_acceleration Accelerometer [x, y, z] m/s²
 */
void ros_publish_imu(const float *orientation,
                     const float *angular_velocity,
                     const float *linear_acceleration);

/**
 * @brief Check for new base velocity command
 * @param velocities Output array [left, right] rad/s
 * @return true if new command available
 */
bool ros_get_base_cmd(float *velocities);

/**
 * @brief Check for new arm position command
 * @param positions Output array [j1..j6] rad
 * @return true if new command available
 */
bool ros_get_arm_cmd(float *positions);

/**
 * @brief Check for new camera position command
 * @param positions Output array [pan, tilt] rad
 * @return true if new command available
 */
bool ros_get_camera_cmd(float *positions);

#endif /* ROS_H */
