
#include <ros/ros.h>
#include "extended_kalman_filter/noise_odometry.hpp"
#include "extended_kalman_filter/sensor_fusion.hpp"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ekf_node");
    noise::Odometry noise_odometry;
    odom_to_imu::SensorFusion sensor_fusion;
    ros::spin();
    return 0;
}