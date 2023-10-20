cmake_minimum_required(VERSION 3.0.2)
project(potbot_pcl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  cv_bridge
  pcl_ros
  dynamic_reconfigure
  # beego_control
  #dynamixel_sdk
  #dynamixel_sdk_examples
  #obstacle_detection_2019
  visualization_msgs
  #autonomous_mobile_robot
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(-std=c++14)
