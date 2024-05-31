/**
 * @file emcl.h
 * @author Toshiki Nakamura
 * @brief C++ implementation of emcl(mcl with expansion resetting)
 * @copyright Copyright (c) 2024
 */

#ifndef EMCL_EMCL_H
#define EMCL_EMCL_H

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <optional>
#include <pcl_ros/point_cloud.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

#include "emcl_ros/DynReconfConfig.h"

#include "utils/odom_model.h"
#include "utils/particle.h"
#include "utils/pose.h"

/**
 * @struct EMCLParam
 * @brief EMCL parameters
 */
struct EMCLParam
{
  bool use_cloud = false;
  int particle_num = 0;
  int reset_counter = 0;
  int reset_count_limit = 0;
  int laser_step = 0;
  float init_x = 0.0;
  float init_y = 0.0;
  float init_yaw = 0.0;
  float init_position_dev = 0.0;
  float init_orientation_dev = 0.0;
  float likelihood_th = 0.0;
  float expansion_position_dev = 0.0;
  float expansion_orientation_dev = 0.0;
  float sensor_noise_ratio = 0.0;
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
 * @struct ScanParam
 * @brief Scan parameters
 */
struct ScanParam
{
  float range_min = 0.0;
  float range_max = 0.0;
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
   * @brief Callback function for cloud
   * @param msg Cloud
   */
  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * @brief Callback function for dynamic reconfigure
   * @param config Dynamic reconfigure
   * @param level Level
   */
  void dyn_reconf_callback(emcl_ros::DynReconfConfig &config, uint32_t level);

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
   * @brief Callback function for odom
   * @param msg Odom
   */
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

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
   * @brief Calculate the normal distribution
   * @param mean Mean
   * @param stddev Standard deviation
   * @return float Normal distribution
   */
  float norm_dist(const float mean, const float stddev);

  /**
   * @brief Client function for map
   */
  void get_map(void);

  /**
   * @brief Update the motion model
   */
  void motion_update(void);

  /**
   * @brief Normalize the angle
   * @param angle Angle
   * @return float Normalized angle
   */
  float normalize_angle(float angle);

  /**
   * @brief Update the observation model
   * @param cloud Cloud
   * @return float Average likelihood
   */
  float calc_average_likelihood(const sensor_msgs::PointCloud2 &cloud);

  /**
   * @brief Update the observation model
   * @param laser_scan Laser scan
   * @return float Average likelihood
   */
  float calc_average_likelihood(const sensor_msgs::LaserScan &laser_scan);

  /**
   * @brief Calculate the total likelihood
   * @return float Total likelihood
   */
  float calc_total_likelihood(void);

  /**
   * @brief Estimate the pose by the weighted mean
   */
  void estimate_pose(void);

  /**
   * @brief Normalize the belief
   */
  void normalize_belief(void);

  /**
   * @brief Reset by expansion
   */
  void expansion_reset(void);

  /**
   * @brief Resampling the particles
   */
  void resampling(void);

  /**
   * @brief Broadcast the odom state
   */
  void broadcast_map_to_odom_tf(void);

  /**
   * @brief Publish the estimated pose
   */
  void publish_estimated_pose(void);

  /**
   * @brief Publish the particles
   */
  void publish_particles(void);

  Pose emcl_pose_;
  EMCLParam emcl_param_;
  OdomModel odom_model_;
  OdomModelParam odom_model_param_;
  ScanParam scan_param_;

  std::string odom_frame_id_;

  std::vector<Particle> particles_;

  bool use_dynamic_reconfigure_ = false;

  std::random_device rd_;
  std::mt19937 gen_{rd_()};

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher emcl_pose_pub_;
  ros::Publisher particle_cloud_pub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  nav_msgs::OccupancyGrid map_;
  std::optional<Pose> prev_odom_;
  std::optional<Pose> last_odom_;

  dynamic_reconfigure::Server<emcl_ros::DynReconfConfig> dyn_reconf_server_;
};

#endif  // EMCL_EMCL_H
