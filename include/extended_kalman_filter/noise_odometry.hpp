#ifndef __NOISE_ODOMETRY_HPP__
#define __NOISE_ODOMETRY_HPP__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <random>

namespace noise {
class RobotState {
public:
  double x_;
  double y_;
  double yaw_;

  double normalize_angle(double angle){
    while(angle > M_PI) { angle -= 2.0 * M_PI; }
    while(angle < -M_PI) { angle += 2.0 * M_PI; }
    return angle;
  }

  RobotState() : x_(0.0), y_(0.0), yaw_(0.0) {}
  ~RobotState() {}
};

class Odometry {
private:
  RobotState state_;

  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;

  ros::Publisher odom_pub_;

  nav_msgs::Path noise_odom_path_msg_;

  tf::TransformBroadcaster odom_broadcaster_;

  double AddNoise(double mean, double stddev) {
    static std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(generator);
  }

  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;
    
    if(linear_velocity == 0.0 && angular_velocity == 0.0) {
      return;
    }

    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;  

    double delta_x = linear_velocity * cos(state_.yaw_) * dt 
                     + AddNoise(0.0, 0.0);
    double delta_y = linear_velocity * sin(state_.yaw_) * dt 
                     + AddNoise(0.0, 0.0);
    double delta_theta = angular_velocity * dt + delta_theta 
                     + AddNoise(0.0, 0.005);

    state_.x_ += delta_x;
    state_.y_ += delta_y;
    state_.yaw_ += delta_theta;

    state_.yaw_ = state_.normalize_angle(state_.yaw_);

    PublishOdom(linear_velocity + AddNoise(0.0, 0.0), 
                linear_velocity + AddNoise(0.0, 0.0), 
                angular_velocity + AddNoise(0.0, 0.005));
  }

  void PublishOdom(double noise_linear_velocity_x, 
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

public:
  Odometry() {
    cmd_vel_sub_ =
      nh_.subscribe("cmd_vel", 1, &Odometry::CmdVelCallback, this);
    odom_pub_ = 
      nh_.advertise<nav_msgs::Odometry>("noise_odom", 1);
  }
};
}
#endif