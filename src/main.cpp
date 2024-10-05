
#include <ros/ros.h>
#include "extended_kalman_filter/SensorFusion.hpp"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ekf_node");
    SensorFusion sensorFusion;
    ros::spin();
    return 0;
}