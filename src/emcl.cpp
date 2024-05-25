/**
 * @file emcl.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of emcl(mcl with expansion resetting)
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <vector>

#include "emcl/emcl.h"

EMCL::EMCL() : private_nh_("~")
{
  load_params();

  emcl_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/emcl_pose", 1);
  particle_cloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1);
  initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &EMCL::initial_pose_callback, this);
  laser_scan_sub_ = nh_.subscribe("/scan", 1, &EMCL::laser_scan_callback, this);
  map_sub_ = nh_.subscribe("/map", 1, &EMCL::map_callback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &EMCL::odom_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  print_params();

  particles_.reserve(emcl_param_.particle_num);
  odom_model_ = OdomModel(odom_model_param_.ff, odom_model_param_.fr, odom_model_param_.rf, odom_model_param_.rr);
  initialize(emcl_param_.init_x, emcl_param_.init_y, emcl_param_.init_yaw);
}

void EMCL::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  initialize(msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
}

void EMCL::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) { laser_scan_ = *msg; }

void EMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) { map_ = *msg; }

void EMCL::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (last_odom_.has_value())
    prev_odom_ = last_odom_;
  last_odom_ = *msg;
}

void EMCL::process(void)
{
  ros::Rate loop_rate(emcl_param_.hz);

  while (ros::ok())
  {
    if (map_.has_value() && prev_odom_.has_value() && laser_scan_.has_value())
      localize();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void EMCL::initialize(const float init_x, const float init_y, const float init_yaw)
{
  emcl_pose_.set(init_x, init_y, init_yaw);
  particles_.clear();
  for (int i = 0; i < emcl_param_.particle_num; i++)
    particles_.push_back(Particle(
        norm_dist(init_x, emcl_param_.init_x_dev), norm_dist(init_y, emcl_param_.init_y_dev),
        norm_dist(init_yaw, emcl_param_.init_yaw_dev)));
  reset_weight();
  ROS_WARN("Initialized");
}

float EMCL::norm_dist(const float mean, const float stddev)
{
  std::normal_distribution<> norm_dist(mean, stddev);
  return norm_dist(gen_);
}

void EMCL::reset_weight(void)
{
  for (auto &p : particles_)
    p.set_weight(1.0 / particles_.size());
}

void EMCL::broadcast_odom_state(void)
{
  static tf2_ros::TransformBroadcaster odom_state_broadcaster;

  const float map_to_base_yaw = emcl_pose_.yaw();
  const float map_to_base_x = emcl_pose_.x();
  const float map_to_base_y = emcl_pose_.y();

  const float odom_to_base_yaw = tf2::getYaw(last_odom_.value().pose.pose.orientation);
  const float odom_to_base_x = last_odom_.value().pose.pose.position.x;
  const float odom_to_base_y = last_odom_.value().pose.pose.position.y;

  const float map_to_odom_yaw = normalize_angle(map_to_base_yaw - odom_to_base_yaw);
  const float map_to_odom_x =
      map_to_base_x - odom_to_base_x * cos(map_to_odom_yaw) + odom_to_base_y * sin(map_to_odom_yaw);
  const float map_to_odom_y =
      map_to_base_y - odom_to_base_x * sin(map_to_odom_yaw) - odom_to_base_y * cos(map_to_odom_yaw);

  tf2::Quaternion map_to_odom_quat;
  map_to_odom_quat.setRPY(0, 0, map_to_odom_yaw);

  geometry_msgs::TransformStamped odom_state;

  odom_state.header.stamp = ros::Time::now();

  odom_state.header.frame_id = map_.value().header.frame_id;
  odom_state.child_frame_id = last_odom_.value().header.frame_id;

  odom_state.transform.translation.x = isnan(map_to_odom_x) ? 0.0 : map_to_odom_x;
  odom_state.transform.translation.y = isnan(map_to_odom_y) ? 0.0 : map_to_odom_y;
  tf2::convert(map_to_odom_quat, odom_state.transform.rotation);

  odom_state_broadcaster.sendTransform(odom_state);
}

float EMCL::normalize_angle(float angle)
{
  while (M_PI < angle)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void EMCL::localize(void)
{
  // Update the particles
  motion_update();

  // Update the observation model
  const float average_likelihood = calc_average_likelihood();

  // Estimate the pose by the weighted mean
  estimate_pose();
  broadcast_odom_state();
  publish_estimated_pose();
  publish_particles();

  // reset or resampling
  ROS_INFO_STREAM_THROTTLE(1.0, "average_likelihood = " << std::fixed << std::setprecision(4) << average_likelihood);
  if (average_likelihood < emcl_param_.likelihood_th && emcl_param_.reset_counter < emcl_param_.reset_count_limit)
  {
    expansion_resetting();
    emcl_param_.reset_counter++;
  }
  else
  {
    resampling();
    emcl_param_.reset_counter = 0;
  }
}

void EMCL::motion_update(void)
{
  const float last_yaw = tf2::getYaw(last_odom_.value().pose.pose.orientation);
  const float prev_yaw = tf2::getYaw(prev_odom_.value().pose.pose.orientation);

  const float dx = last_odom_.value().pose.pose.position.x - prev_odom_.value().pose.pose.position.x;
  const float dy = last_odom_.value().pose.pose.position.y - prev_odom_.value().pose.pose.position.y;
  const float dyaw = normalize_angle(last_yaw - prev_yaw);

  const float length = hypot(dx, dy);
  const float direction = normalize_angle(atan2(dy, dx) - prev_yaw);

  odom_model_.set_SD(length, dyaw);

  for (auto &p : particles_)
    p.pose_.move(length, direction, dyaw, odom_model_.get_fw_noise(), odom_model_.get_rot_noise());
}

float EMCL::calc_average_likelihood(void)
{
  // Update the observation model
  for (auto &p : particles_)
  {
    const float likelihood =
        p.likelihood(map_.value(), laser_scan_.value(), emcl_param_.sensor_noise_ratio, emcl_param_.laser_step);
    p.set_weight(p.weight() * likelihood);
  }

  // Count the number of valid laser scan data
  int valid_laser_size = 0;
  for (int i = 0; i < laser_scan_.value().ranges.size(); i += emcl_param_.laser_step)
  {
    if (laser_scan_.value().ranges[i] < laser_scan_.value().range_min ||
        laser_scan_.value().range_max < laser_scan_.value().ranges[i])
      continue;
    valid_laser_size++;
  }

  return calc_total_likelihood() / (valid_laser_size * particles_.size());
}

float EMCL::calc_total_likelihood(void)
{
  float sum = 0.0;
  for (const auto &p : particles_)
    sum += p.weight();
  return sum;
}

// Estimate the pose by the weighted mean
void EMCL::estimate_pose(void)
{
  normalize_belief();
  float x_mean = 0.0;
  float y_mean = 0.0;
  float dyaw_mean = 0.0;
  for (const auto &p : particles_)
  {
    x_mean += p.pose_.x() * p.weight();
    y_mean += p.pose_.y() * p.weight();
    dyaw_mean += normalize_angle(emcl_pose_.yaw() - p.pose_.yaw()) * p.weight();
  }
  emcl_pose_.set(x_mean, y_mean, emcl_pose_.yaw() - dyaw_mean);
}

void EMCL::normalize_belief(void)
{
  const float weight_sum = calc_total_likelihood();
  for (auto &p : particles_)
    p.set_weight(p.weight() / weight_sum);
}

void EMCL::expansion_resetting(void)
{
  for (auto &p : particles_)
    p.pose_.set(
        norm_dist(p.pose_.x(), emcl_param_.expansion_x_dev), norm_dist(p.pose_.y(), emcl_param_.expansion_y_dev),
        norm_dist(p.pose_.yaw(), emcl_param_.expansion_yaw_dev));
  reset_weight();
}

void EMCL::resampling(void)
{
  std::vector<float> accumulation_weight;
  accumulation_weight.push_back(particles_.front().weight());
  for (int i = 1; i < particles_.size(); i++)
    accumulation_weight.push_back(accumulation_weight.back() + particles_[i].weight());

  const std::vector<Particle> old_particles_(particles_);
  const float step = 1.0 / static_cast<float>(particles_.size());

  std::uniform_real_distribution<float> uniform_dist(0.0, 1.0);
  for (int i = 0; i < particles_.size(); i++)
  {
    const float darts = uniform_dist(gen_);
    for (int j = 0; j < particles_.size(); j++)
    {
      if (darts < accumulation_weight[j])
      {
        particles_[i] = old_particles_[j];
        break;
      }
    }
  }
  reset_weight();
}

void EMCL::publish_estimated_pose(void)
{
  geometry_msgs::PoseWithCovarianceStamped emcl_pose_msg;
  emcl_pose_msg.pose.pose.position.x = emcl_pose_.x();
  emcl_pose_msg.pose.pose.position.y = emcl_pose_.y();

  tf2::Quaternion q;
  q.setRPY(0, 0, emcl_pose_.yaw());
  tf2::convert(q, emcl_pose_msg.pose.pose.orientation);

  emcl_pose_msg.header.frame_id = map_.value().header.frame_id;
  emcl_pose_msg.header.stamp = ros::Time::now();
  emcl_pose_pub_.publish(emcl_pose_msg);
}

void EMCL::publish_particles(void)
{
  geometry_msgs::PoseArray particle_cloud_msg;
  particle_cloud_msg.poses.reserve(emcl_param_.particle_num);
  geometry_msgs::Pose pose;

  for (const auto &particle : particles_)
  {
    pose.position.x = particle.pose_.x();
    pose.position.y = particle.pose_.y();

    tf2::Quaternion q;
    q.setRPY(0, 0, particle.pose_.yaw());
    tf2::convert(q, pose.orientation);

    particle_cloud_msg.poses.emplace_back(pose);
  }

  particle_cloud_msg.header.frame_id = map_.value().header.frame_id;
  particle_cloud_msg.header.stamp = ros::Time::now();
  particle_cloud_pub_.publish(particle_cloud_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "emcl");
  EMCL emcl;
  emcl.process();

  return 0;
}
