/**
 * @file parameters.cpp
 * @author Toshiki Nakamura
 * @brief Load and print parameters
 * @copyright Copyright (c) 2024
 */

#include <vector>

#include "emcl/emcl.h"

void EMCL::load_params(void)
{
  // EMCL
  // - a -
  private_nh_.param<float>("alpha_th", alpha_th_, 0.0017);
  // - e -
  private_nh_.param<float>("expansion_x_dev", expansion_x_dev_, 0.05);
  private_nh_.param<float>("expansion_y_dev", expansion_y_dev_, 0.05);
  private_nh_.param<float>("expansion_yaw_dev", expansion_yaw_dev_, 0.01);
  // - f -
  private_nh_.param<bool>("flag_broadcast", flag_broadcast_, true);
  private_nh_.param<bool>("flag_init_noise", flag_init_noise_, true);
  // - h -
  private_nh_.param<int>("hz", hz_, 10);
  // - i -
  private_nh_.param<std::vector<float>>("ignore_angle_range_list", ignore_angle_range_list_, {0.0, 0.0, 0.0});
  private_nh_.param<float>("init_x", init_x_, 0.0);
  private_nh_.param<float>("init_x_dev", init_x_dev_, 0.5);
  private_nh_.param<float>("init_y", init_y_, 0.0);
  private_nh_.param<float>("init_y_dev", init_y_dev_, 0.65);
  private_nh_.param<float>("init_yaw", init_yaw_, 0.0);
  private_nh_.param<float>("init_yaw_dev", init_yaw_dev_, 0.5);
  private_nh_.param<bool>("is_visible", is_visible_, true);
  // - l -
  private_nh_.param<int>("laser_step", laser_step_, 10);
  // - m -
  private_nh_.param<float>("move_dist_th", move_dist_th_, 0.025);
  // - p -
  private_nh_.param<int>("particle_num", particle_num_, 420);
  // - r -
  private_nh_.param<int>("reset_count_limit", reset_count_limit_, 5);
  // - s -
  private_nh_.param<float>("sensor_noise_ratio", sensor_noise_ratio_, 0.03);

  // OdomModel
  // - f -
  private_nh_.param<float>("ff", ff_, 0.17);
  private_nh_.param<float>("fr", fr_, 0.0005);
  // - r -
  private_nh_.param<float>("rf", rf_, 0.13);
  private_nh_.param<float>("rr", rr_, 0.2);
}

void EMCL::print_params(void)
{
  ROS_INFO("EMCL Parameters:");
  ROS_INFO_STREAM("  alpha_th: " << alpha_th_);
  ROS_INFO_STREAM("  expansion_x_dev: " << expansion_x_dev_);
  ROS_INFO_STREAM("  expansion_y_dev: " << expansion_y_dev_);
  ROS_INFO_STREAM("  expansion_yaw_dev: " << expansion_yaw_dev_);
  ROS_INFO_STREAM("  flag_broadcast: " << flag_broadcast_);
  ROS_INFO_STREAM("  flag_init_noise: " << flag_init_noise_);
  ROS_INFO_STREAM("  hz: " << hz_);
  ROS_INFO("  ignore_angle_range_list:");
  for (const auto i : ignore_angle_range_list_)
    ROS_INFO_STREAM("    " << i);
  ROS_INFO_STREAM("  init_x: " << init_x_);
  ROS_INFO_STREAM("  init_x_dev: " << init_x_dev_);
  ROS_INFO_STREAM("  init_y: " << init_y_);
  ROS_INFO_STREAM("  init_y_dev: " << init_y_dev_);
  ROS_INFO_STREAM("  init_yaw: " << init_yaw_);
  ROS_INFO_STREAM("  init_yaw_dev: " << init_yaw_dev_);
  ROS_INFO_STREAM("  is_visible: " << is_visible_);
  ROS_INFO_STREAM("  laser_step: " << laser_step_);
  ROS_INFO_STREAM("  move_dist_th: " << move_dist_th_);
  ROS_INFO_STREAM("  particle_num: " << particle_num_);
  ROS_INFO_STREAM("  reset_count_limit: " << reset_count_limit_);
  ROS_INFO_STREAM("  sensor_noise_ratio: " << sensor_noise_ratio_);

  ROS_INFO("OdomModel Parameters:");
  ROS_INFO_STREAM("  ff: " << ff_);
  ROS_INFO_STREAM("  fr: " << fr_);
  ROS_INFO_STREAM("  rf: " << rf_);
  ROS_INFO_STREAM("  rr: " << rr_);
}
