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
  private_nh_.param<float>("alpha_th", emcl_param_.alpha_th, 0.0017);
  // - e -
  private_nh_.param<float>("expansion_x_dev", emcl_param_.expansion_x_dev, 0.05);
  private_nh_.param<float>("expansion_y_dev", emcl_param_.expansion_y_dev, 0.05);
  private_nh_.param<float>("expansion_yaw_dev", emcl_param_.expansion_yaw_dev, 0.01);
  // - f -
  private_nh_.param<bool>("flag_broadcast", flag_broadcast_, true);
  private_nh_.param<bool>("flag_init_noise", emcl_param_.flag_init_noise, true);
  // - h -
  private_nh_.param<int>("hz", emcl_param_.hz, 10);
  // - i -
  private_nh_.param<float>("init_x", emcl_param_.init_x, 0.0);
  private_nh_.param<float>("init_x_dev", emcl_param_.init_x_dev, 0.5);
  private_nh_.param<float>("init_y", emcl_param_.init_y, 0.0);
  private_nh_.param<float>("init_y_dev", emcl_param_.init_y_dev, 0.65);
  private_nh_.param<float>("init_yaw", emcl_param_.init_yaw, 0.0);
  private_nh_.param<float>("init_yaw_dev", emcl_param_.init_yaw_dev, 0.5);
  private_nh_.param<bool>("is_visible", is_visible_, true);
  // - l -
  private_nh_.param<int>("laser_step", emcl_param_.laser_step, 10);
  // - m -
  private_nh_.param<float>("move_dist_th", emcl_param_.move_dist_th, 0.025);
  // - p -
  private_nh_.param<int>("particle_num", emcl_param_.particle_num, 420);
  // - r -
  private_nh_.param<int>("reset_count_limit", emcl_param_.reset_count_limit, 5);
  // - s -
  private_nh_.param<float>("sensor_noise_ratio", emcl_param_.sensor_noise_ratio, 0.03);

  // OdomModel
  // - f -
  private_nh_.param<float>("ff", odom_model_param_.ff, 0.17);
  private_nh_.param<float>("fr", odom_model_param_.fr, 0.0005);
  // - r -
  private_nh_.param<float>("rf", odom_model_param_.rf, 0.13);
  private_nh_.param<float>("rr", odom_model_param_.rr, 0.2);
}

void EMCL::print_params(void)
{
  ROS_INFO("EMCL Parameters:");
  ROS_INFO_STREAM("  alpha_th: " << emcl_param_.alpha_th);
  ROS_INFO_STREAM("  expansion_x_dev: " << emcl_param_.expansion_x_dev);
  ROS_INFO_STREAM("  expansion_y_dev: " << emcl_param_.expansion_y_dev);
  ROS_INFO_STREAM("  expansion_yaw_dev: " << emcl_param_.expansion_yaw_dev);
  ROS_INFO_STREAM("  flag_broadcast: " << flag_broadcast_);
  ROS_INFO_STREAM("  flag_init_noise: " << emcl_param_.flag_init_noise);
  ROS_INFO_STREAM("  hz: " << emcl_param_.hz);
  ROS_INFO_STREAM("  init_x: " << emcl_param_.init_x);
  ROS_INFO_STREAM("  init_x_dev: " << emcl_param_.init_x_dev);
  ROS_INFO_STREAM("  init_y: " << emcl_param_.init_y);
  ROS_INFO_STREAM("  init_y_dev: " << emcl_param_.init_y_dev);
  ROS_INFO_STREAM("  init_yaw: " << emcl_param_.init_yaw);
  ROS_INFO_STREAM("  init_yaw_dev: " << emcl_param_.init_yaw_dev);
  ROS_INFO_STREAM("  is_visible: " << is_visible_);
  ROS_INFO_STREAM("  laser_step: " << emcl_param_.laser_step);
  ROS_INFO_STREAM("  move_dist_th: " << emcl_param_.move_dist_th);
  ROS_INFO_STREAM("  particle_num: " << emcl_param_.particle_num);
  ROS_INFO_STREAM("  reset_count_limit: " << emcl_param_.reset_count_limit);
  ROS_INFO_STREAM("  sensor_noise_ratio: " << emcl_param_.sensor_noise_ratio);

  ROS_INFO("OdomModel Parameters:");
  ROS_INFO_STREAM("  ff: " << odom_model_param_.ff);
  ROS_INFO_STREAM("  fr: " << odom_model_param_.fr);
  ROS_INFO_STREAM("  rf: " << odom_model_param_.rf);
  ROS_INFO_STREAM("  rr: " << odom_model_param_.rr);
}
