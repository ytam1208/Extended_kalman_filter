#ifndef __SENSOR_FUSION_HPP__
#define __SENSOR_FUSION_HPP__

#include <random>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include "extended_kalman_filter/extended_kalman_filter.hpp"

namespace odom_to_imu {
    
constexpr size_t kDIM_X = 3;     // State vector dimension: [x, y, yaw]
constexpr size_t kDIM_Z = 1;     // Measurement vector dimension

class SensorFusion {
private:
    ekf::ExtendedKalmanFilter<kDIM_X, kDIM_Z> ekf_;
    ros::Time last_time_;
    
    ros::NodeHandle nh_;

    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher ekf_traj_pub_;
    ros::Publisher ori_traj_pub_;
    ros::Publisher noise_ori_traj_pub_;

    nav_msgs::Path ekf_odom_path_msg_;
    nav_msgs::Path odom_path_msg_;
    nav_msgs::Path noise_odom_path_msg_;

    tf::TransformBroadcaster odom_broadcaster_;

    std::string odom_frame_id_;
    std::string parent_frame_id_;
    std::string ekf_parent_frame_id_;
    
    std::normal_distribution<double> noise_;
    std::default_random_engine generator_;

    double CalculateYaw(const nav_msgs::Odometry& odom_msg);
    void PublishPath(const ekf::float32_t x, 
                     const ekf::float32_t y, 
                     const ekf::float32_t yaw_rad,
                     const std::string& frame_id, 
                     nav_msgs::Path& path_msg);

    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void PublishOdomPath(const nav_msgs::Odometry& odom_msg);
    void PublishEkfPath(void);
    
public:
    SensorFusion();
    ~SensorFusion();
};
}

#endif