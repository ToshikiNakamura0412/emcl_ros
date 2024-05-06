/**
 * @file odom_model.h
 * @author Toshiki Nakamura
 * @brief Class of OdomModel
 * @copyright Copyright (c) 2024
 */

#ifndef UTILS_ODOM_MODEL_H
#define UTILS_ODOM_MODEL_H

#include <random>

class OdomModel
{
public:
  OdomModel() {}
  OdomModel(const float ff, const float fr, const float rf, const float rr);
  OdomModel &operator=(const OdomModel &model);

  void set_dev(const float length, const float angle);
  float get_fw_noise();
  float get_rot_noise();

private:
  float fw_var_per_fw_;
  float fw_var_per_rot_;
  float rot_var_per_fw_;
  float rot_var_per_rot_;
  float fw_dev_;
  float rot_dev_;

  std::normal_distribution<> std_norm_dist_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
};

#endif  // UTILS_ODOM_MODEL_H
