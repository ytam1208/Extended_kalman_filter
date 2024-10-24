#include "extended_kalman_filter/sensor_fusion.hpp"

namespace odom_to_imu {

SensorFusion::SensorFusion() {
  odom_frame_id_ = "odom";
  ekf_parent_frame_id_ = "fake_base_link";
  
  imu_sub_ = nh_.subscribe("imu", 
                            10, 
                            &SensorFusion::ImuCallback, this);
  odom_sub_ = nh_.subscribe("odom", 
                            1, 
                            &SensorFusion::OdomCallback, this); 
  noise_odom_sub_ = nh_.subscribe("noise_odom", 
                            1, 
                            &SensorFusion::NoiseOdomCallback, this);

  ekf_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ekf_odom", 1);

  gt_traj_pub_ = nh_.advertise<nav_msgs::Path>("/ground_truth_traj", 1);
  ekf_traj_pub_ = nh_.advertise<nav_msgs::Path>("/ekf_traj", 1);
  noise_ori_traj_pub_ = nh_.advertise<nav_msgs::Path>("/noise_odom_traj", 1);
}

SensorFusion::~SensorFusion() {}

double SensorFusion::CalculateYaw(const nav_msgs::Odometry& odom_msg) {
  geometry_msgs::Quaternion quat = odom_msg.pose.pose.orientation;

  // Convert quaternion to yaw
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(quat, tf_quat);
  
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

void SensorFusion::PublishPath(const ekf::float32_t x, 
                               const ekf::float32_t y, 
                               const ekf::float32_t yaw_rad,
                               const std::string& frame_id, 
                               nav_msgs::Path& path_msg) {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = frame_id;

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  geometry_msgs::Quaternion odom_quat = 
      tf::createQuaternionMsgFromYaw(yaw_rad);
  pose.pose.orientation = odom_quat;
  
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = frame_id;;
  path_msg.poses.push_back(pose);
}

void SensorFusion::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ekf_.Correct(yaw, 0.01);
}

void SensorFusion::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  PublishOdomPath(*msg, gt_path_msg_);

  gt_traj_pub_.publish(gt_path_msg_);
}

void SensorFusion::NoiseOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time_).toSec();
  last_time_ = current_time;

  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  double yaw_angular_v = msg->twist.twist.angular.z;
  
  ekf::Vector<kDIM_X> u;
  u << vx, vy, yaw_angular_v;
  
  ekf_.Predict(u, dt);
  PublishEkfPath();
  PublishOdomPath(*msg, noise_odom_path_msg_);

  ekf_traj_pub_.publish(ekf_odom_path_msg_);
  noise_ori_traj_pub_.publish(noise_odom_path_msg_);
}

void SensorFusion::PublishOdomPath(const nav_msgs::Odometry& odom_msg,         
                                   nav_msgs::Path& path_msg) {
  double x = odom_msg.pose.pose.position.x;
  double y = odom_msg.pose.pose.position.y;
  double yaw = CalculateYaw(odom_msg);

  PublishPath(x, y, yaw, "odom", path_msg);
}

void SensorFusion::PublishEkfPath(void) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = odom_frame_id_;   
  odom_msg.child_frame_id = ekf_parent_frame_id_;

  odom_msg.pose.pose.position.x = ekf_.vecX()(0);
  odom_msg.pose.pose.position.y = ekf_.vecX()(1);
  odom_msg.pose.pose.position.z = 0.0;  

  geometry_msgs::Quaternion odom_quat = 
    tf::createQuaternionMsgFromYaw(ekf_.vecX()(2));
  odom_msg.pose.pose.orientation = odom_quat;
  ekf_odom_pub_.publish(odom_msg);

  tf::Transform odom_trans;
  odom_trans.setOrigin(tf::Vector3(ekf_.vecX()(0), ekf_.vecX()(1), 0.0));

  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(odom_quat, tf_quat);  

  odom_trans.setRotation(tf_quat);

  odom_broadcaster_.sendTransform(
      tf::StampedTransform(odom_trans, ros::Time::now(), 
                            odom_frame_id_, ekf_parent_frame_id_));
  
  PublishPath(ekf_.vecX()(0), ekf_.vecX()(1), ekf_.vecX()(2), 
              "odom", 
              ekf_odom_path_msg_);
}

}