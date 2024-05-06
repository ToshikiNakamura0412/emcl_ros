/**
 * @file particle.h
 * @author Toshiki Nakamura
 * @brief Class of Particle
 * @copyright Copyright (c) 2024
 */

#ifndef UTILS_PARTICLE_H
#define UTILS_PARTICLE_H

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

#include "utils/pose.h"

class Particle
{
public:
  Particle();
  Particle(const float x, const float y, const float yaw, const float weight);
  Particle &operator=(const Particle &p);

  void set_weight(const float weight);
  float weight() const { return weight_; }
  float likelihood(
      const nav_msgs::OccupancyGrid &map, const sensor_msgs::LaserScan &laser, const float sensor_noise_ratio,
      const int laser_step, const std::vector<float> &ignore_angle_range_list);

  Pose pose_;

private:
  bool is_ignore_angle(float angle, const std::vector<float> &ignore_angle_range_list);
  bool in_map(const int grid_index, const int map_data_size);
  int xy_to_grid_index(const float x, const float y, const nav_msgs::MapMetaData &map_info);
  float norm_pdf(const float x, const float mean, const float stddev);
  float calc_dist_to_wall(
      float x, float y, const float laser_angle, const nav_msgs::OccupancyGrid &map, const float laser_range,
      const float sensor_noise_ratio);

  float weight_;
};

#endif  // UTILS_PARTICLE_H
