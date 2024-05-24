/**
 * @file particle.cpp
 * @author Toshiki Nakamura
 * @brief Class of Particle
 * @copyright Copyright (c) 2024
 */

#include <vector>

#include "utils/particle.h"

Particle::Particle(void) : pose_(0.0, 0.0, 0.0) { weight_ = 0.0; }

Particle::Particle(const float x, const float y, const float yaw) : pose_(x, y, yaw) { weight_ = 0.0; }

Particle &Particle::operator=(const Particle &p)
{
  pose_ = p.pose_;
  weight_ = p.weight_;
  return *this;
}

void Particle::set_weight(const float weight) { weight_ = weight; }

float Particle::likelihood(
    const nav_msgs::OccupancyGrid &map, const sensor_msgs::LaserScan &laser, const float sensor_noise_ratio,
    const int laser_step)
{
  float L = 0.0;

  for (int i = 0; i < laser.ranges.size(); i += laser_step)
  {
    if (laser.ranges[i] < laser.range_min || laser.range_max < laser.ranges[i])
      continue;

    const float angle = i * laser.angle_increment + laser.angle_min;
    const float range =
        calc_dist_to_wall(pose_.x(), pose_.y(), angle + pose_.yaw(), map, laser.ranges[i], sensor_noise_ratio);
    L += norm_pdf(range, laser.ranges[i], laser.ranges[i] * sensor_noise_ratio);
  }

  return L;
}

float Particle::calc_dist_to_wall(
    float x, float y, const float laser_angle, const nav_msgs::OccupancyGrid &map, const float laser_range,
    const float sensor_noise_ratio)
{
  const float search_step = map.info.resolution;
  const float search_limit = laser_range;

  for (float dist = 0.0; dist < search_limit; dist += search_step)
  {
    x += search_step * cos(laser_angle);
    y += search_step * sin(laser_angle);

    const int grid_index = xy_to_grid_index(x, y, map.info);

    if (!in_map(grid_index, map.data.size()))
      return search_limit * 2.0;
    else if (map.data[grid_index] == -1)
      return search_limit * 2.0;
    else if (map.data[grid_index] == 100)
      return dist;
  }

  return search_limit * sensor_noise_ratio * 5.0;
}

float Particle::norm_pdf(const float x, const float mean, const float stddev)
{
  return 1.0 / sqrt(2.0 * M_PI * pow(stddev, 2.0)) * exp(-pow((x - mean), 2.0) / (2.0 * pow(stddev, 2.0)));
}

int Particle::xy_to_grid_index(const float x, const float y, const nav_msgs::MapMetaData &map_info)
{
  const int index_x = static_cast<int>(floor((x - map_info.origin.position.x) / map_info.resolution));
  const int index_y = static_cast<int>(floor((y - map_info.origin.position.y) / map_info.resolution));
  return index_x + (index_y * map_info.width);
}

bool Particle::in_map(const int grid_index, const int map_data_size)
{
  if (0 <= grid_index && grid_index < map_data_size)
    return true;
  else
    return false;
}
