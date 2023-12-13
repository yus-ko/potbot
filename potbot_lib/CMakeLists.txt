cmake_minimum_required(VERSION 3.0.2)
project(potbot_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # sensor_msgs
  # message_generation
  # cv_bridge
  pcl_ros
  # dynamic_reconfigure
  # beego_control
  #dynamixel_sdk
  #dynamixel_sdk_examples
  #obstacle_detection_2019
  # visualization_msgs
  #autonomous_mobile_robot
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS
  # DEPENDS eigen opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Utility.cpp
  src/PotentialField.cpp
  src/PathPlanner.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-std=c++14)