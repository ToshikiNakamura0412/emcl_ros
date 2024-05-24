/**
 * @file odom_model.h
 * @author Toshiki Nakamura
 * @brief Class of OdomModel
 * @copyright Copyright (c) 2024
 */

#ifndef UTILS_ODOM_MODEL_H
#define UTILS_ODOM_MODEL_H

#include <random>

/**
 * @class OdomModel
 * @brief Class of OdomModel
*/
class OdomModel
{
public:
  /**
   * @brief Construct a new OdomModel object
  */
  OdomModel(void) {}

  /**
   * @brief Construct a new OdomModel object
   * @param ff Standard deviation of forward noise per forward
   * @param fr Standard deviation of forward noise per rotation
   * @param rf Standard deviation of rotation noise per forward
   * @param rr Standard deviation of rotation noise per rotation
  */
  OdomModel(const float ff, const float fr, const float rf, const float rr);

  /**
   * @brief Overload of operator=
   * @param model OdomModel object
   * @return OdomModel& OdomModel object
  */
  OdomModel &operator=(const OdomModel &model);

  /**
   * @brief Set the standard deviation
   * @param length Length of the movement
   * @param angle Angle of the movement
  */
  void set_SD(const float length, const float angle);

  /**
   * @brief Get the forward noise
   * @return float Forward noise
  */
  float get_fw_noise(void) { return std_norm_dist_(engine_) * fw_stddev_; }

  /**
   * @brief Get the rotation noise
   * @return float Rotation noise
  */
  float get_rot_noise(void) { return std_norm_dist_(engine_) * rot_stddev_; }

private:
  float fw_var_per_fw_;
  float fw_var_per_rot_;
  float rot_var_per_fw_;
  float rot_var_per_rot_;
  float fw_stddev_;
  float rot_stddev_;

  std::normal_distribution<> std_norm_dist_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
};

#endif  // UTILS_ODOM_MODEL_H
