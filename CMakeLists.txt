cmake_minimum_required(VERSION 3.0.2)
project(extended_kalman_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  eigen_conversions
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf geometry_msgs 
  DEPENDS EIGEN3
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
)

add_executable(ekf_node 
  src/main.cpp
  src/sensor_fusion.cpp
  src/noise_odometry.cpp
)

target_link_libraries(ekf_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)