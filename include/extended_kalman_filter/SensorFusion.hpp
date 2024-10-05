#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "extended_kalman_filter/ekf.hpp"

constexpr size_t DIM_X = 3;     // State vector dimension: [x, y, yaw]
constexpr size_t DIM_Z = 1;     // Measurement vector dimension

class SensorFusion {
private:
    ekf::Extended_KalmanFilter<DIM_X, DIM_Z> ekf;
    ros::Time last_time;
    
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster;
    
    std::string odom_frame_id_;
    std::string parent_frame_id_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat);

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        ekf.correct(yaw, 0.01);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double dt = (ros::Time::now() - last_time).toSec();
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double yaw_rate = msg->twist.twist.angular.z;

        ekf::Vector<DIM_X> u;
        u << vx, vy, yaw_rate;

        ekf.predict(u, dt);
        last_time = ros::Time::now();

        publishOdometry();
    }

    void publishOdometry() {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = odom_frame_id_;   
        odom_msg.child_frame_id = parent_frame_id_;

        odom_msg.pose.pose.position.x = ekf.vecX()(0);
        odom_msg.pose.pose.position.y = ekf.vecX()(1);
        odom_msg.pose.pose.position.z = 0.0;  

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(ekf.vecX()(2));
        odom_msg.pose.pose.orientation = odom_quat;

        odom_pub_.publish(odom_msg);

        tf::Transform odom_trans;
        odom_trans.setOrigin(tf::Vector3(ekf.vecX()(0), ekf.vecX()(1), 0.0));

        tf::Quaternion tf_quat;
        tf::quaternionMsgToTF(odom_quat, tf_quat);  

        odom_trans.setRotation(tf_quat);

        odom_broadcaster.sendTransform(
            tf::StampedTransform(odom_trans, ros::Time::now(), odom_frame_id_, parent_frame_id_));
    }

public:
    SensorFusion() {
        odom_frame_id_ = "odom";
        parent_frame_id_ = "fake_base_link";

        imu_sub_ = nh_.subscribe("imu", 100, &SensorFusion::imuCallback, this);
        odom_sub_ = nh_.subscribe("odom", 100, &SensorFusion::odomCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ekf_odom", 100);
        last_time = ros::Time::now();
    }
};
