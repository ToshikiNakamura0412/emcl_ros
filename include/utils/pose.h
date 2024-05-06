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

class Pose
{
public:
  Pose();
  Pose(const float x, const float y, const float yaw);
  Pose &operator=(const Pose &pose);
  Pose &operator/=(const float a);

  void set(const float x, const float y, const float yaw);
  float x() const { return x_; }
  float y() const { return y_; }
  float yaw() const { return yaw_; }
  void move(float length, float direction, float rotation, const float fw_noise, const float rot_noise);
  void normalize_angle();

private:
  float x_;    // [m]
  float y_;    // [m]
  float yaw_;  // [rad]
};

#endif  // UTILS_POSE_H
