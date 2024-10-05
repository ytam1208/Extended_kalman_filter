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

  double AddNoise(double mean, double stddev);
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  double NormalizeAngle(double angle);
  void PublishOdom(double noise_linear_velocity_x, 
                   double noise_linear_velocity_y, 
                   double noise_angular_velocity);
public:
  Odometry();
  ~Odometry();
};
}
#endif