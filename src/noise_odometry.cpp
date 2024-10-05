#include "extended_kalman_filter/noise_odometry.hpp"

namespace noise {

Odometry::Odometry() {
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &Odometry::CmdVelCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/noise_odom", 1);
}

Odometry::~Odometry() {}

double Odometry::AddNoise(double mean, double stddev) {
  static std::default_random_engine generator;
  std::normal_distribution<double> distribution(mean, stddev);
  return distribution(generator);
}

void Odometry::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  double linear_velocity = msg->linear.x + AddNoise(0.0, 0.0);
  double angular_velocity = msg->angular.z + AddNoise(0.0, 0.025);
  
  if (linear_velocity == 0.0 && angular_velocity == 0.0) {
    return;
  }

  static ros::Time last_time = ros::Time::now();
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = current_time;  

  double delta_x = linear_velocity * cos(state_.yaw_) * dt;
  double delta_y = linear_velocity * sin(state_.yaw_) * dt;
  double delta_yaw = angular_velocity * dt;

  state_.x_ += delta_x;
  state_.y_ += delta_y;
  state_.yaw_ += delta_yaw;

  state_.yaw_ = NormalizeAngle(state_.yaw_);

  PublishOdom(linear_velocity, linear_velocity, angular_velocity);  
}

double Odometry::NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void Odometry::PublishOdom(double noise_linear_velocity_x, 
                           double noise_linear_velocity_y, 
                           double noise_angular_velocity) {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "noise_base_link";

  odom.pose.pose.position.x = state_.x_;
  odom.pose.pose.position.y = state_.y_;
  odom.pose.pose.position.z = 0.0;

  odom.twist.twist.linear.x = noise_linear_velocity_x;
  odom.twist.twist.linear.y = noise_linear_velocity_y;
  odom.twist.twist.angular.z = noise_angular_velocity;

  geometry_msgs::Quaternion odom_quat = 
    tf::createQuaternionMsgFromYaw(state_.yaw_);
  odom.pose.pose.orientation = odom_quat;
  
  odom_pub_.publish(odom);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(state_.x_, state_.y_, 0.0));
  transform.setRotation(
    tf::Quaternion(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w));

  odom_broadcaster_.sendTransform(
    tf::StampedTransform(transform, ros::Time::now(), 
                         odom.header.frame_id, odom.child_frame_id));
}

}  // namespace noise