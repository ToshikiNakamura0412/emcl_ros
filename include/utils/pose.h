/**
 * @file pose.h
 * @author Toshiki Nakamura
 * @brief Class of Pose
 * @copyright Copyright (c) 2024
 */

#ifndef UTILS_POSE_H
#define UTILS_POSE_H

#include <cmath>
#include <utility>

/**
 * @class Pose
 * @brief Class of Pose
 */
class Pose
{
public:
  /**
   * @brief Construct a new Pose object
   */
  Pose(void);

  /**
   * @brief Construct a new Pose object
   * @param x x-coordinate
   * @param y y-coordinate
   * @param yaw yaw
   */
  Pose(const float x, const float y, const float yaw);

  /**
   * @brief Copy constructor
   * @param pose Pose object
   */
  Pose &operator=(const Pose &pose);

  /**
   * @brief Division operator
   * @param a float
   */
  Pose &operator/=(const float a);

  /**
   * @brief Set the pose
   * @param x x-coordinate
   * @param y y-coordinate
   * @param yaw yaw
   */
  void set(const float x, const float y, const float yaw);

  /**
   * @brief Get the x-coordinate
   * @return float x-coordinate
   */
  float x() const { return x_; }

  /**
   * @brief Get the y-coordinate
   * @return float y-coordinate
   */
  float y() const { return y_; }

  /**
   * @brief Get the yaw
   * @return float yaw
   */
  float yaw() const { return yaw_; }

  /**
   * @brief Move the pose
   * @param length length
   * @param direction direction
   * @param rotation rotation
   * @param fw_noise forward noise
   * @param rot_noise rotation noise
   */
  void move(float length, float direction, float rotation, const float fw_noise, const float rot_noise);

private:
  float x_;    // [m]
  float y_;    // [m]
  float yaw_;  // [rad]
};

#endif  // UTILS_POSE_H
