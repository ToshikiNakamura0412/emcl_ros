/**
 * @file pose.cpp
 * @author Toshiki Nakamura
 * @brief Class of Pose
 * @copyright Copyright (c) 2024
 */

#include <utility>

#include "utils/pose.h"

Pose::Pose(void)
{
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
}

Pose::Pose(const float x, const float y, const float yaw)
{
  x_ = x;
  y_ = y;
  yaw_ = yaw;
}

Pose &Pose::operator=(const Pose &pose)
{
  x_ = pose.x_;
  y_ = pose.y_;
  yaw_ = pose.yaw_;
  return *this;
}

Pose &Pose::operator/=(const float a)
{
  x_ /= a;
  y_ /= a;
  yaw_ /= a;
  return *this;
}

void Pose::set(const float x, const float y, const float yaw)
{
  x_ = x;
  y_ = y;
  yaw_ = yaw;
}

void Pose::move(float length, float direction, float rotation, const float fw_noise, const float rot_noise)
{
  length += fw_noise;
  direction += rot_noise;
  rotation += rot_noise;

  x_ += length * cos(direction + yaw_);
  y_ += length * sin(direction + yaw_);
  yaw_ += rotation;
}
