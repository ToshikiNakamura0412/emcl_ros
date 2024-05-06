/**
 * @file emcl.h
 * @author Toshiki Nakamura
 * @brief C++ implementation of emcl(mcl with expansion resetting)
 * @copyright Copyright (c) 2024
 */

#ifndef EMCL_EMCL_H
#define EMCL_EMCL_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <optional>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

#include "utils/odom_model.h"
#include "utils/particle.h"
#include "utils/pose.h"

class EMCL
{
public:
  EMCL(void);
  void process(void);

private:
  void load_params(void);
  void print_params(void);
  void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  float get_median(std::vector<float> &data);
  float norm_rv(const float mean, const float stddev);
  float normalize_angle(float angle);
  void initialize(const float init_x, const float init_y, const float init_yaw);
  void reset_weight(void);
  void broadcast_odom_state(void);
  void localize(void);
  void motion_update(void);
  void observation_update(void);
  void estimate_pose(void);
  void mean_pose(void);
  void weighted_mean_pose(void);
  void max_weight_pose(void);
  void median_pose(void);
  void normalize_belief(void);
  void expansion_resetting(void);
  void resampling(void);
  void publish_estimated_pose(void);
  void publish_particles(void);
  float calc_marginal_likelihood(void);

  int hz_;
  int particle_num_;
  int reset_counter_;
  int reset_count_limit_;
  int laser_step_;
  float move_dist_th_;
  float init_x_;
  float init_y_;
  float init_yaw_;
  float init_x_dev_;
  float init_y_dev_;
  float init_yaw_dev_;
  float alpha_th_;
  float expansion_x_dev_;
  float expansion_y_dev_;
  float expansion_yaw_dev_;
  float sensor_noise_ratio_;
  Pose emcl_pose_;
  OdomModel odom_model_;

  std::vector<Particle> particles_;
  std::vector<float> ignore_angle_range_list_;

  bool flag_move_;
  bool flag_init_noise_;
  bool flag_broadcast_;
  bool is_visible_;

  float ff_;
  float fr_;
  float rf_;
  float rr_;

  unsigned int seed_;

  std::random_device seed_gen_;
  std::default_random_engine engine_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher emcl_pose_pub_;
  ros::Publisher particle_cloud_pub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber odom_sub_;

  std::optional<sensor_msgs::LaserScan> laser_scan_;
  std::optional<nav_msgs::OccupancyGrid> map_;
  std::optional<nav_msgs::Odometry> initial_odom_;
  nav_msgs::Odometry prev_odom_;
  nav_msgs::Odometry last_odom_;
  geometry_msgs::PoseStamped emcl_pose_msg_;
  geometry_msgs::PoseArray particle_cloud_msg_;
};

#endif  // EMCL_EMCL_H
