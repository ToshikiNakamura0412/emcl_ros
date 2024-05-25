/**
 * @file emcl.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of emcl(mcl with expansion resetting)
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <vector>

#include "emcl/emcl.h"

EMCL::EMCL() : private_nh_("~"), flag_move_(false)
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
  particle_cloud_msg_.poses.reserve(emcl_param_.particle_num);
  odom_model_ = OdomModel(odom_model_param_.ff, odom_model_param_.fr, odom_model_param_.rf, odom_model_param_.rr);
  initialize(emcl_param_.init_x, emcl_param_.init_y, emcl_param_.init_yaw);
}

void EMCL::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  initialize(msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
  flag_move_ = false;
}

void EMCL::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) { laser_scan_ = *msg; }

void EMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) { map_ = *msg; }

void EMCL::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (!initial_odom_.has_value())
  {
    initial_odom_ = *msg;
    prev_odom_ = *msg;
  }
  else
  {
    prev_odom_ = last_odom_;
  }
  last_odom_ = *msg;

  if (!flag_move_)
  {
    const float dist_from_init_x = prev_odom_.pose.pose.position.x - initial_odom_.value().pose.pose.position.x;
    const float dist_from_init_y = prev_odom_.pose.pose.position.y - initial_odom_.value().pose.pose.position.y;
    if (emcl_param_.move_dist_th < hypot(dist_from_init_x, dist_from_init_y))
      flag_move_ = true;
  }
}

void EMCL::process(void)
{
  ros::Rate loop_rate(emcl_param_.hz);

  while (ros::ok())
  {
    if (map_.has_value() && initial_odom_.has_value() && laser_scan_.has_value())
    {
      if (flag_move_)
      {
        localize();
      }
      else
      {
        broadcast_odom_state(Pose(emcl_param_.init_x, emcl_param_.init_y, emcl_param_.init_yaw));
        publish_estimated_pose(Pose(emcl_param_.init_x, emcl_param_.init_y, emcl_param_.init_yaw));
        publish_particles();
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void EMCL::initialize(const float init_x, const float init_y, const float init_yaw)
{
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

void EMCL::broadcast_odom_state(const Pose &pose)
{
  if (flag_broadcast_)
  {
    static tf2_ros::TransformBroadcaster odom_state_broadcaster;

    const float map_to_base_yaw = pose.yaw();
    const float map_to_base_x = pose.x();
    const float map_to_base_y = pose.y();

    const float odom_to_base_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
    const float odom_to_base_x = last_odom_.pose.pose.position.x;
    const float odom_to_base_y = last_odom_.pose.pose.position.y;

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
    odom_state.child_frame_id = last_odom_.header.frame_id;

    odom_state.transform.translation.x = isnan(map_to_odom_x) ? 0.0 : map_to_odom_x;
    odom_state.transform.translation.y = isnan(map_to_odom_y) ? 0.0 : map_to_odom_y;
    tf2::convert(map_to_odom_quat, odom_state.transform.rotation);

    odom_state_broadcaster.sendTransform(odom_state);
  }
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
  const Pose emcl_pose = estimate_pose();
  broadcast_odom_state(emcl_pose);
  publish_estimated_pose(emcl_pose);
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
  const float last_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
  const float prev_yaw = tf2::getYaw(prev_odom_.pose.pose.orientation);

  const float dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
  const float dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
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
  int laser_size = laser_scan_.value().ranges.size();
  for (const auto &range : laser_scan_.value().ranges)
  {
    if (range < laser_scan_.value().range_min || laser_scan_.value().range_max < range)
      laser_size--;
  }

  return calc_total_likelihood() / ((laser_size / emcl_param_.laser_step) * particles_.size());
}

float EMCL::calc_total_likelihood(void)
{
  float sum = 0.0;
  for (const auto &p : particles_)
    sum += p.weight();
  return sum;
}

// Estimate the pose by the weighted mean
Pose EMCL::estimate_pose(void)
{
  normalize_belief();
  float x_mean = 0.0;
  float y_mean = 0.0;
  float yaw_mean = particles_[0].pose_.yaw();
  float max_weight = particles_[0].weight();
  for (const auto &p : particles_)
  {
    x_mean += p.pose_.x() * p.weight();
    y_mean += p.pose_.y() * p.weight();

    if (max_weight < p.weight())
    {
      yaw_mean = p.pose_.yaw();
      max_weight = p.weight();
    }
  }
  return Pose(x_mean, y_mean, yaw_mean);
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
        particles_[i].set_weight(1.0 / particles_.size());
        break;
      }
    }
  }
}

void EMCL::publish_estimated_pose(const Pose &pose)
{
  emcl_pose_msg_.pose.pose.position.x = pose.x();
  emcl_pose_msg_.pose.pose.position.y = pose.y();

  tf2::Quaternion q;
  q.setRPY(0, 0, pose.yaw());
  tf2::convert(q, emcl_pose_msg_.pose.pose.orientation);

  emcl_pose_msg_.header.frame_id = map_.value().header.frame_id;
  emcl_pose_msg_.header.stamp = ros::Time::now();
  emcl_pose_pub_.publish(emcl_pose_msg_);
}

void EMCL::publish_particles(void)
{
  particle_cloud_msg_.poses.clear();
  geometry_msgs::Pose pose;

  for (const auto &particle : particles_)
  {
    pose.position.x = particle.pose_.x();
    pose.position.y = particle.pose_.y();

    tf2::Quaternion q;
    q.setRPY(0, 0, particle.pose_.yaw());
    tf2::convert(q, pose.orientation);

    particle_cloud_msg_.poses.push_back(pose);
  }

  particle_cloud_msg_.header.frame_id = map_.value().header.frame_id;
  particle_cloud_msg_.header.stamp = ros::Time::now();
  particle_cloud_pub_.publish(particle_cloud_msg_);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "emcl");
  EMCL emcl;
  emcl.process();

  return 0;
}
