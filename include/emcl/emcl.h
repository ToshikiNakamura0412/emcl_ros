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

/**
 * @struct EMCLParam
 * @brief EMCL parameters
*/
struct EMCLParam
{
  int hz = 0;
  int particle_num = 0;
  int reset_counter = 0;
  int reset_count_limit = 0;
  int laser_step = 0;
  float move_dist_th = 0.0;
  float init_x = 0.0;
  float init_y = 0.0;
  float init_yaw = 0.0;
  float init_x_dev = 0.0;
  float init_y_dev = 0.0;
  float init_yaw_dev = 0.0;
  float alpha_th = 0.0;
  float expansion_x_dev = 0.0;
  float expansion_y_dev = 0.0;
  float expansion_yaw_dev = 0.0;
  float sensor_noise_ratio = 0.0;
  bool flag_init_noise = false;
};

/**
 * @struct OdomModelParam
 * @brief Odom model parameters
*/
struct OdomModelParam
{
  float ff = 0.0;
  float fr = 0.0;
  float rf = 0.0;
  float rr = 0.0;
};

/**
 * @class EMCL
 * @brief Class of EMCL
*/
class EMCL
{
public:
  /**
   * @brief Construct a new EMCL object
  */
  EMCL(void);

  /**
   * @brief Construct a new EMCL object
  */
  void process(void);

private:
  /**
   * @brief Load the parameters
  */
  void load_params(void);

  /**
   * @brief Print the parameters
  */
  void print_params(void);

  /**
   * @brief Callback function for initial pose
   * @param msg Initial pose
  */
  void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  /**
   * @brief Callback function for laser scan
   * @param msg Laser scan
  */
  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

  /**
   * @brief Callback function for map
   * @param msg Map
  */
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  /**
   * @brief Callback function for odom
   * @param msg Odom
  */
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief Get the median
   * @param data Data
  */
  float get_median(std::vector<float> &data);

  /**
   * @brief Calculate the normal distribution
   * @param mean Mean
   * @param stddev Standard deviation
   * @return float Normal distribution
  */
  float norm_dist(const float mean, const float stddev);

  /**
   * @brief Normalize the angle
   * @param angle Angle
   * @return float Normalized angle
  */
  float normalize_angle(float angle);

  /**
   * @brief Initialize the particles
   * @param init_x Initial x
   * @param init_y Initial y
   * @param init_yaw Initial yaw
  */
  void initialize(const float init_x, const float init_y, const float init_yaw);

  /**
   * @brief Reset the weight (uniform distribution)
  */
  void reset_weight(void);

  /**
   * @brief Broadcast the odom state
  */
  void broadcast_odom_state(void);

  /**
   * @brief Publish the estimated pose
  */
  void localize(void);

  /**
   * @brief Update the motion model
  */
  void motion_update(void);

  /**
   * @brief Update the observation model
  */
  void observation_update(void);

  /**
   * @brief Estimate the pose
  */
  void estimate_pose(void);

  /**
   * @brief Estimate the pose by the mean
  */
  void mean_pose(void);

  /**
   * @brief Estimate the pose by the weighted mean
  */
  void weighted_mean_pose(void);

  /**
   * @brief Estimate the pose by the max weight
  */
  void max_weight_pose(void);

  /**
   * @brief Estimate the pose by the median
  */
  void median_pose(void);

  /**
   * @brief Normalize the belief
  */
  void normalize_belief(void);

  /**
   * @brief Resetting by expansion
  */
  void expansion_resetting(void);

  /**
   * @brief Resampling the particles
  */
  void resampling(void);

  /**
   * @brief Publish the estimated pose
  */
  void publish_estimated_pose(void);

  /**
   * @brief Publish the particles
  */
  void publish_particles(void);

  /**
   * @brief Calculate the marginal likelihood
   * @return float Marginal likelihood
  */
  float calc_marginal_likelihood(void);

  Pose emcl_pose_;
  EMCLParam emcl_param_;
  OdomModel odom_model_;
  OdomModelParam odom_model_param_;

  std::vector<Particle> particles_;

  bool flag_move_;
  bool flag_broadcast_;
  bool is_visible_;

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
  geometry_msgs::PoseWithCovarianceStamped emcl_pose_msg_;
  geometry_msgs::PoseArray particle_cloud_msg_;
};

#endif // EMCL_EMCL_H
