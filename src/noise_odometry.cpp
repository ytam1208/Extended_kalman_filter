// #include "extended_kalman_filter/noise_odometry.hpp"

// noise::RobotState::RobotState() : x_(0.0), y_(0.0), yaw_(0.0) {}

// noise::RobotState::~RobotState() {}

// noise::Odometry::Odometry() {
//   parent_frame_id_ = "odom";
//   odom_frame_id_ = "noise_base_link";

//   cmd_vel_sub_ = nh_.subscribe("cmd_vel", 
//                                1, 
//                                &noise::Odometry::CmdvelCallback, this);
//   odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
// }

// noise::Odometry::~Odometry() {}

// double noise::Odometry::AddNoise(const double& mean, const double& stddev) {
//   static std::default_random_engine generator;
//   std::normal_distribution<double> distribution(mean, stddev);
//   return distribution(generator);
// }

// void noise::Odometry::CmdvelCallback(
//   const geometry_msgs::Twist::ConstPtr& msg) {
//   double linear_velocity = msg->linear.x;
//   double angular_velocity = msg->angular.z;
  
//   if(msg->linear.x == 0.0 && angular_velocity == 0.0) {
//     return;
//   }

//   static ros::Time last_time = ros::Time::now();
//   ros::Time current_time = ros::Time::now();
//   double dt = (current_time - last_time).toSec();
//   last_time = current_time;  

//   double delta_x = linear_velocity * cos(state_.yaw_) * dt 
//                     + AddNoise(0.0, 0.0025);;
//   double delta_y = linear_velocity * sin(state_.yaw_) * dt 
//                     + AddNoise(0.0, 0.0025);;
//   double delta_theta = angular_velocity * dt + delta_theta 
//                     + AddNoise(0.0, 0.01);

//   state_.x_ += delta_x;
//   state_.y_ += delta_y;
//   state_.yaw_ += delta_theta;

//   state_.yaw_ = NormalizeAngle(state_.yaw_);

//   PublishOdom(linear_velocity + AddNoise(0.0, 0.0025), 
//               linear_velocity + AddNoise(0.0, 0.0025), 
//               angular_velocity + AddNoise(0.0, 0.05));
// }

// double noise::Odometry::NormalizeAngle(double angle){
//   while(angle > M_PI) { angle -= 2.0 * M_PI; }
//   while(angle < -M_PI) { angle += 2.0 * M_PI; }
//   return angle;
// }

// void noise::Odometry::PublishOdom(
//                   const double& noise_linear_velocity_x, 
//                   const double& noise_linear_velocity_y, 
//                   const double& noise_angular_velocity) {
//   nav_msgs::Odometry odom;
//   odom.header.stamp = ros::Time::now();
//   odom.header.frame_id = parent_frame_id_;
//   odom.child_frame_id = odom_frame_id_;

//   odom.pose.pose.position.x = state_.x_;
//   odom.pose.pose.position.y = state_.y_;
//   odom.pose.pose.position.z = 0.0;

//   odom.twist.twist.linear.x = noise_linear_velocity_x;
//   odom.twist.twist.linear.y = noise_linear_velocity_y;
//   odom.twist.twist.angular.z = noise_angular_velocity;

//   geometry_msgs::Quaternion odom_quat = 
//     tf::createQuaternionMsgFromYaw(state_.yaw_);
//   odom.pose.pose.orientation = odom_quat;
  
//   odom_pub_.publish(odom);

//   tf::Transform transform;
//   transform.setOrigin(tf::Vector3(state_.x_, state_.y_, 0.0));
//   transform.setRotation(
//     tf::Quaternion(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w));

//   odom_broadcaster_.sendTransform(
//     tf::StampedTransform(transform, 
//                          ros::Time::now(), 
//                          parent_frame_id_, 
//                          odom_frame_id_));
// }