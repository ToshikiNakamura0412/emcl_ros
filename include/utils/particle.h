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

/**
 * @class Particle
 * @brief Class of Particle
 */
class Particle
{
public:
  /**
   * @brief Construct a new Particle object
   */
  Particle(void);

  /**
   * @brief Construct a new Particle object
   * @param x x-coordinate
   * @param y y-coordinate
   * @param yaw yaw
   */
  Particle(const float x, const float y, const float yaw);

  /**
   * @brief Overload of operator=
   * @param p Particle
   * @return Particle&
   */
  Particle &operator=(const Particle &p);

  /**
   * @brief Set the weight object
   * @param weight weight
   */
  void set_weight(const float weight);

  /**
   * @brief Get the weight object
   * @return float weight
   */
  float weight(void) const { return weight_; }

  /**
   * @brief Calculate the likelihood
   * @param map map
   * @param laser laser
   * @param sensor_noise_ratio sensor noise ratio
   * @param laser_step laser step
   * @param ignore_angle_range_list ignore angle range list
   * @return float likelihood
   */
  float likelihood(
      const nav_msgs::OccupancyGrid &map, const sensor_msgs::LaserScan &laser, const float sensor_noise_ratio,
      const int laser_step);

  Pose pose_;

private:
  /**
   * @brief Calculate the distance to the wall
   * @param x x-coordinate
   * @param y y-coordinate
   * @param laser_angle laser angle
   * @param map map
   * @param laser_range laser range
   * @param sensor_noise_ratio sensor noise ratio
   * @return float distance to the wall
   */
  float calc_dist_to_wall(
      float x, float y, const float laser_angle, const nav_msgs::OccupancyGrid &map, const float laser_range,
      const float sensor_noise_ratio);

  /**
   * @brief Calculate the normal distribution
   * @param x x
   * @param mean mean
   * @param stddev standard deviation
   * @return float normal distribution
   */
  float norm_pdf(const float x, const float mean, const float stddev);

  /**
   * @brief Convert the x-y coordinate to the grid index
   * @param x x-coordinate
   * @param y y-coordinate
   * @param map_info map info
   * @return int grid index
   */
  int xy_to_grid_index(const float x, const float y, const nav_msgs::MapMetaData &map_info);

  /**
   * @brief Check if the grid index is in the map
   * @param grid_index grid index
   * @param map_data_size map data size
   * @return true if the grid index is in the map
   * @return false if the grid index is not in the map
   */
  bool in_map(const int grid_index, const int map_data_size);

  float weight_;
};

#endif  // UTILS_PARTICLE_H
