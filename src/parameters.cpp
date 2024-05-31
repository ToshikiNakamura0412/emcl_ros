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
  // - e -
  private_nh_.param<float>("expansion_position_dev", emcl_param_.expansion_position_dev, 0.07);
  private_nh_.param<float>("expansion_orientation_dev", emcl_param_.expansion_orientation_dev, 0.2);
  // - i -
  private_nh_.param<float>("init_x", emcl_param_.init_x, 0.0);
  private_nh_.param<float>("init_y", emcl_param_.init_y, 0.0);
  private_nh_.param<float>("init_yaw", emcl_param_.init_yaw, 0.0);
  private_nh_.param<float>("init_position_dev", emcl_param_.init_position_dev, 0.1);
  private_nh_.param<float>("init_orientation_dev", emcl_param_.init_orientation_dev, 0.05);
  // - l -
  private_nh_.param<int>("laser_step", emcl_param_.laser_step, 4);
  private_nh_.param<float>("likelihood_th", emcl_param_.likelihood_th, 0.002);
  // - p -
  private_nh_.param<int>("particle_num", emcl_param_.particle_num, 420);
  // - r -
  private_nh_.param<int>("reset_count_limit", emcl_param_.reset_count_limit, 3);
  // - s -
  private_nh_.param<float>("sensor_noise_ratio", emcl_param_.sensor_noise_ratio, 0.03);
  // - u -
  private_nh_.param<bool>("use_cloud", emcl_param_.use_cloud, false);

  // OdomModel
  // - f -
  private_nh_.param<float>("ff", odom_model_param_.ff, 0.17);
  private_nh_.param<float>("fr", odom_model_param_.fr, 0.0005);
  // - r -
  private_nh_.param<float>("rf", odom_model_param_.rf, 0.13);
  private_nh_.param<float>("rr", odom_model_param_.rr, 0.2);

  // Scan
  // - r -
  private_nh_.param<float>("range_min", scan_param_.range_min, 0.12);
  private_nh_.param<float>("range_max", scan_param_.range_max, 3.5);

  // Common
  // - u -
  private_nh_.param<bool>("use_dynamic_reconfigure", use_dynamic_reconfigure_, false);
}

void EMCL::print_params(void)
{
  ROS_INFO("EMCL Parameters:");
  ROS_INFO_STREAM("  expansion_position_dev: " << emcl_param_.expansion_position_dev);
  ROS_INFO_STREAM("  expansion_orientation_dev: " << emcl_param_.expansion_orientation_dev);
  ROS_INFO_STREAM("  init_x: " << emcl_param_.init_x);
  ROS_INFO_STREAM("  init_y: " << emcl_param_.init_y);
  ROS_INFO_STREAM("  init_yaw: " << emcl_param_.init_yaw);
  ROS_INFO_STREAM("  init_position_dev: " << emcl_param_.init_position_dev);
  ROS_INFO_STREAM("  init_orientation_dev: " << emcl_param_.init_orientation_dev);
  ROS_INFO_STREAM("  laser_step: " << emcl_param_.laser_step);
  ROS_INFO_STREAM("  likelihood_th: " << emcl_param_.likelihood_th);
  ROS_INFO_STREAM("  particle_num: " << emcl_param_.particle_num);
  ROS_INFO_STREAM("  reset_count_limit: " << emcl_param_.reset_count_limit);
  ROS_INFO_STREAM("  sensor_noise_ratio: " << emcl_param_.sensor_noise_ratio);
  ROS_INFO_STREAM("  use_cloud: " << emcl_param_.use_cloud);

  ROS_INFO("OdomModel Parameters:");
  ROS_INFO_STREAM("  ff: " << odom_model_param_.ff);
  ROS_INFO_STREAM("  fr: " << odom_model_param_.fr);
  ROS_INFO_STREAM("  rf: " << odom_model_param_.rf);
  ROS_INFO_STREAM("  rr: " << odom_model_param_.rr);

  ROS_INFO("Scan Parameters:");
  ROS_INFO_STREAM("  range_min: " << scan_param_.range_min);
  ROS_INFO_STREAM("  range_max: " << scan_param_.range_max);

  ROS_INFO("Common Parameters:");
  ROS_INFO_STREAM("  use_dynamic_reconfigure: " << use_dynamic_reconfigure_);
}

void EMCL::dyn_reconf_callback(emcl_ros::DynReconfConfig &config, uint32_t level)
{
  if (!use_dynamic_reconfigure_)
    return;

  ROS_WARN("Update parameters:");
  if (abs(emcl_param_.expansion_position_dev - config.expansion_position_dev) > FLT_EPSILON)
  {
    emcl_param_.expansion_position_dev = config.expansion_position_dev;
    ROS_WARN_STREAM("  expansion_position_dev: " << emcl_param_.expansion_position_dev);
  }
  if (abs(emcl_param_.expansion_orientation_dev - config.expansion_orientation_dev) > FLT_EPSILON)
  {
    emcl_param_.expansion_orientation_dev = config.expansion_orientation_dev;
    ROS_WARN_STREAM("  expansion_orientation_dev: " << emcl_param_.expansion_orientation_dev);
  }
  if (emcl_param_.laser_step != config.laser_step)
  {
    emcl_param_.laser_step = config.laser_step;
    ROS_WARN_STREAM("  laser_step: " << emcl_param_.laser_step);
  }
  if (abs(emcl_param_.likelihood_th - config.likelihood_th) > FLT_EPSILON)
  {
    emcl_param_.likelihood_th = config.likelihood_th;
    ROS_WARN_STREAM("  likelihood_th: " << emcl_param_.likelihood_th);
  }
  if (emcl_param_.reset_count_limit != config.reset_count_limit)
  {
    emcl_param_.reset_count_limit = config.reset_count_limit;
    ROS_WARN_STREAM("  reset_count_limit: " << emcl_param_.reset_count_limit);
  }
  if (abs(emcl_param_.sensor_noise_ratio - config.sensor_noise_ratio) > FLT_EPSILON)
  {
    emcl_param_.sensor_noise_ratio = config.sensor_noise_ratio;
    ROS_WARN_STREAM("  sensor_noise_ratio: " << emcl_param_.sensor_noise_ratio);
  }
  if (abs(odom_model_param_.ff - config.ff) > FLT_EPSILON)
  {
    odom_model_param_.ff = config.ff;
    ROS_WARN_STREAM("  ff: " << odom_model_param_.ff);
  }
  if (abs(odom_model_param_.fr - config.fr) > FLT_EPSILON)
  {
    odom_model_param_.fr = config.fr;
    ROS_WARN_STREAM("  fr: " << odom_model_param_.fr);
  }
  if (abs(odom_model_param_.rf - config.rf) > FLT_EPSILON)
  {
    odom_model_param_.rf = config.rf;
    ROS_WARN_STREAM("  rf: " << odom_model_param_.rf);
  }
  if (abs(odom_model_param_.rr - config.rr) > FLT_EPSILON)
  {
    odom_model_param_.rr = config.rr;
    ROS_WARN_STREAM("  rr: " << odom_model_param_.rr);
  }
}
