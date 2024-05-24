/**
 * @file odom_model.cpp
 * @author Toshiki Nakamura
 * @brief Class of OdomModel
 * @copyright Copyright (c) 2024
 */

#include "utils/odom_model.h"

OdomModel::OdomModel(const float ff, const float fr, const float rf, const float rr)
    : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0), fw_stddev_(0.0), rot_stddev_(0.0)
{
  fw_var_per_fw_ = pow(ff, 2.0);
  fw_var_per_rot_ = pow(fr, 2.0);
  rot_var_per_fw_ = pow(rf, 2.0);
  rot_var_per_rot_ = pow(rr, 2.0);
}

OdomModel &OdomModel::operator=(const OdomModel &model)
{
  fw_var_per_fw_ = model.fw_var_per_fw_;
  fw_var_per_rot_ = model.fw_var_per_rot_;
  rot_var_per_fw_ = model.rot_var_per_fw_;
  rot_var_per_rot_ = model.rot_var_per_rot_;
  fw_stddev_ = model.fw_stddev_;
  rot_stddev_ = model.rot_stddev_;
  return *this;
}

void OdomModel::set_SD(const float length, const float angle)
{
  fw_stddev_ = sqrt(fabs(length) * fw_var_per_fw_ + fabs(angle) * fw_var_per_rot_);
  rot_stddev_ = sqrt(fabs(length) * rot_var_per_fw_ + fabs(angle) * rot_var_per_rot_);
}
