
## This repository is for studying the Kalman filter and contains many experimental parts.
Performance evaluation of the algorithm has not been conducted yet.

---
### 1. Test trajectory 
(Blue: Ground Truth, Green: EKF(Encoder-IMU), Red: Odometry(Noise-Encoder) trajectory)

![image](https://github.com/user-attachments/assets/655efeb0-93aa-4a5f-b02d-1931aad6e808)


### 2. Command
```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

rviz

roslaunch extended_kalman_filter ekf.launch
```


### 3. Reference
[1] [Site](https://codingcorner.org/blog/the-kalman-filter/)

[2] [Kalman Filter](https://codingcorner.org/intro-kalman-filter-explained/)

[3] [Extended Kalman Filter](https://codingcorner.org/extended-kalman-filter-explained/)
