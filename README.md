
## This repository is for studying the Kalman filter and contains many experimental parts.
Performance evaluation of the algorithm has not been conducted yet.

---
### 1. Test trajectory 
(Blue: Ground Truth, Green: EKF(Encoder-IMU) trajectory)

![1](https://github.com/user-attachments/assets/19ce8edd-cf68-4190-8da1-e8fb8538e1e9)


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
