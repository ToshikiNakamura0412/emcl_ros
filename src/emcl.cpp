/**
 * @file emcl.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of emcl(mcl with expansion resetting)
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <vector>

#include "emcl/emcl.h"

EMCL::EMCL() : private_nh_("~"), engine_(seed_gen_()), seed_(time(NULL))
{
  load_params();

  emcl_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/emcl_pose", 1);
  particle_cloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1);
  initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &EMCL::initial_pose_callback, this);
  laser_scan_sub_ = nh_.subscribe("/scan", 1, &EMCL::laser_scan_callback, this);
  map_sub_ = nh_.subscribe("/map", 1, &EMCL::map_callback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &EMCL::odom_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  print_params();

  particle_cloud_msg_.poses.reserve(particle_num_);
  odom_model_ = OdomModel(ff_, fr_, rf_, rr_);
  initialize(init_x_, init_y_, init_yaw_);
}

void EMCL::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  initial_pose_ = msg->pose.pose;
}

void EMCL::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  laser_ = *msg;
  flag_laser_ = true;
}

void EMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  map_ = *msg;
  flag_map_ = true;
}

void EMCL::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  prev_odom_ = last_odom_;
  last_odom_ = *msg;
  flag_odom_ = true;

  if (!flag_move_)
  {
    const float dx = prev_odom_.pose.pose.position.x;
    const float dy = prev_odom_.pose.pose.position.y;
    if (move_dist_th_ < hypot(dx, dy))
      flag_move_ = true;
  }
}

void EMCL::process()
{
  ros::Rate loop_rate(hz_);

  while (ros::ok())
  {
    if (flag_map_ && flag_odom_ && flag_laser_)
    {
      if (initial_pose_.has_value())
      {
        initialize(
            initial_pose_.value().position.x, initial_pose_.value().position.y,
            tf2::getYaw(initial_pose_.value().orientation));
      }
      broadcast_odom_state();
      if (flag_move_)
      {
        localize();
      }
      else
      {
        publish_estimated_pose();
        publish_particles();
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void EMCL::initialize(const float init_x, const float init_y, const float init_yaw)
{
  emcl_pose_.set(init_x, init_y, init_yaw);

  Particle particle;
  for (int i = 0; i < particle_num_; i++)
  {
    if (flag_init_noise_)
    {
      const float x = norm_rv(init_x, init_x_dev_);
      const float y = norm_rv(init_y, init_y_dev_);
      const float yaw = norm_rv(init_yaw, init_yaw_dev_);
      particle.pose_.set(x, y, yaw);
      particle.pose_.normalize_angle();
    }
    else
    {
      const float x = init_x;
      const float y = init_y;
      const float yaw = init_yaw;
      particle.pose_.set(x, y, yaw);
      particle.pose_.normalize_angle();
    }
    particles_.push_back(particle);
  }

  reset_weight();

  ROS_WARN("Initialized");
}

float EMCL::norm_rv(const float mean, const float stddev)
{
  std::normal_distribution<> norm_dist(mean, stddev);
  return norm_dist(engine_);
}

void EMCL::reset_weight()
{
  for (auto &p : particles_)
    p.set_weight(1.0 / particles_.size());
}

void EMCL::broadcast_odom_state()
{
  if (flag_broadcast_)
  {
    static tf2_ros::TransformBroadcaster odom_state_broadcaster;

    const float map_to_base_yaw = emcl_pose_.yaw();
    const float map_to_base_x = emcl_pose_.x();
    const float map_to_base_y = emcl_pose_.y();

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

    odom_state.header.frame_id = map_.header.frame_id;
    odom_state.child_frame_id = last_odom_.header.frame_id;

    odom_state.transform.translation.x = map_to_odom_x;
    odom_state.transform.translation.y = map_to_odom_y;
    odom_state.transform.rotation.x = map_to_odom_quat.x();
    odom_state.transform.rotation.y = map_to_odom_quat.y();
    odom_state.transform.rotation.z = map_to_odom_quat.z();
    odom_state.transform.rotation.w = map_to_odom_quat.w();

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

void EMCL::localize()
{
  motion_update();
  observation_update();
  publish_estimated_pose();
  publish_particles();
}

void EMCL::motion_update()
{
  const float last_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
  const float prev_yaw = tf2::getYaw(prev_odom_.pose.pose.orientation);

  const float dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
  const float dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
  const float dyaw = normalize_angle(last_yaw - prev_yaw);

  const float length = hypot(dx, dy);
  const float direction = normalize_angle(atan2(dy, dx) - prev_yaw);

  odom_model_.set_dev(length, dyaw);

  for (auto &p : particles_)
    p.pose_.move(length, direction, dyaw, odom_model_.get_fw_noise(), odom_model_.get_rot_noise());
}

void EMCL::observation_update()
{
  for (auto &p : particles_)
  {
    const float L = p.likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_);
    p.set_weight(p.weight() * L);
  }

  const float alpha = calc_marginal_likelihood() / ((laser_.ranges.size() / laser_step_) * particles_.size());

  if (alpha < alpha_th_ && reset_counter < reset_count_limit_)
  {
    ROS_INFO_STREAM("[resetting] alpha = " << std::fixed << std::setprecision(4) << alpha);
    median_pose();
    expansion_resetting();
    reset_counter++;
  }
  else
  {
    ROS_INFO_STREAM("[resampling] alpha = " << std::fixed << std::setprecision(4) << alpha);
    estimate_pose();
    resampling();
    reset_counter = 0;
  }
}

float EMCL::calc_marginal_likelihood()
{
  float sum = 0.0;
  for (const auto &p : particles_)
    sum += p.weight();

  return sum;
}

void EMCL::estimate_pose()
{
  // mean_pose();
  weighted_mean_pose();
  // max_weight_pose();
  // median_pose();
}

void EMCL::mean_pose()
{
  float x_sum = 0.0;
  float y_sum = 0.0;
  float yaw_sum = 0.0;
  for (const auto &p : particles_)
  {
    x_sum += p.pose_.x();
    y_sum += p.pose_.y();
    yaw_sum += p.pose_.yaw();
  }

  emcl_pose_.set(x_sum, y_sum, yaw_sum);
  emcl_pose_ /= particles_.size();
  emcl_pose_.normalize_angle();
}

void EMCL::weighted_mean_pose()
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

  emcl_pose_.set(x_mean, y_mean, yaw_mean);
}

void EMCL::normalize_belief()
{
  const float weight_sum = calc_marginal_likelihood();

  for (auto &p : particles_)
    p.set_weight(p.weight() / weight_sum);
}

void EMCL::max_weight_pose()
{
  float max_weight = particles_[0].weight();
  emcl_pose_ = particles_[0].pose_;
  for (const auto &p : particles_)
  {
    if (max_weight < p.weight())
    {
      max_weight = p.weight();
      emcl_pose_ = p.pose_;
    }
  }
}

void EMCL::median_pose()
{
  std::vector<float> x_list;
  std::vector<float> y_list;
  std::vector<float> yaw_list;

  for (const auto &p : particles_)
  {
    x_list.push_back(p.pose_.x());
    y_list.push_back(p.pose_.y());
    yaw_list.push_back(p.pose_.yaw());
  }

  const float x_median = get_median(x_list);
  const float y_median = get_median(y_list);
  const float yaw_median = get_median(yaw_list);
  emcl_pose_.set(x_median, y_median, yaw_median);
}

float EMCL::get_median(std::vector<float> &data)
{
  sort(begin(data), end(data));
  if (data.size() % 2 == 1)
    return data[(data.size() - 1) / 2];
  else
    return (data[data.size() / 2 - 1] + data[data.size() / 2]) / 2.0;
}

void EMCL::expansion_resetting()
{
  for (auto &p : particles_)
  {
    const float x = norm_rv(p.pose_.x(), expansion_x_dev_);
    const float y = norm_rv(p.pose_.y(), expansion_y_dev_);
    const float yaw = norm_rv(p.pose_.yaw(), expansion_yaw_dev_);
    p.pose_.set(x, y, yaw);
    p.pose_.normalize_angle();
  }

  reset_weight();
}

void EMCL::resampling()
{
  std::vector<float> accum;
  accum.push_back(particles_[0].weight());
  for (int i = 1; i < particles_.size(); i++)
    accum.push_back(accum.back() + particles_[i].weight());

  const std::vector<Particle> old(particles_);
  const float step = accum.back() / particles_.size();
  const float start =
      static_cast<float>(rand_r(&seed_)) / static_cast<float>(RAND_MAX * step);  // 0 ~ W/N (W: sum of weight)

  std::vector<int> chosen_indexes;
  int tick = 0;
  for (int i = 0; i < particles_.size(); i++)
  {
    while (accum[tick] <= start + i * step)
    {
      tick++;
      if (tick == particles_.size())
      {
        ROS_ERROR("Resampling Failed");
        exit(1);
      }
    }
    chosen_indexes.push_back(tick);
  }

  for (int i = 0; i < particles_.size(); i++)
    particles_[i] = old[chosen_indexes[i]];

  reset_weight();
}

void EMCL::publish_estimated_pose()
{
  emcl_pose_msg_.pose.position.x = emcl_pose_.x();
  emcl_pose_msg_.pose.position.y = emcl_pose_.y();

  tf2::Quaternion q;
  q.setRPY(0, 0, emcl_pose_.yaw());
  tf2::convert(q, emcl_pose_msg_.pose.orientation);

  emcl_pose_msg_.header.frame_id = map_.header.frame_id;
  emcl_pose_msg_.header.stamp = ros::Time::now();
  emcl_pose_pub_.publish(emcl_pose_msg_);
}

void EMCL::publish_particles()
{
  if (is_visible_)
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

    particle_cloud_msg_.header.frame_id = map_.header.frame_id;
    particle_cloud_msg_.header.stamp = ros::Time::now();
    particle_cloud_pub_.publish(particle_cloud_msg_);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "emcl");
  EMCL emcl;
  emcl.process();

  return 0;
}
