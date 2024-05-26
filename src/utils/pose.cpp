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

Pose Pose::operator-(const Pose a)
{
  Pose pose(x_ - a.x_, y_ - a.y_, normalize_angle(yaw_ - a.yaw_));
  return pose;
}

void Pose::set(const float x, const float y, const float yaw)
{
  x_ = x;
  y_ = y;
  yaw_ = yaw;
}

void Pose::move(float length, float direction, float rotation, const float fw_noise, const float rot_noise)
{
  x_ += (length + fw_noise) * cos(direction + rot_noise + yaw_);
  y_ += (length + fw_noise) * sin(direction + rot_noise + yaw_);
  yaw_ += rotation + rot_noise;
  yaw_ = normalize_angle(yaw_);
}

float Pose::normalize_angle(float angle)
{
  while (M_PI < angle)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

bool Pose::nearly_zero(void)
{
  return (fabs(x_) < 1.0e-6 && fabs(y_) < 1.0e-6 && fabs(yaw_) < 1.0e-6);
}
