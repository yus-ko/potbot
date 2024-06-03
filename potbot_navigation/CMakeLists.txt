cmake_minimum_required(VERSION 3.0.2)
project(potbot_navigation)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  potbot_msgs
  potbot_lib
  potbot_controller
  potbot_localmap
  potbot_pathplanner
  potbot_filter
  potbot_pcl
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES navigation
  # CATKIN_DEPENDS 
  # DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation src/node.cpp)
target_link_libraries(navigation ${catkin_LIBRARIES})

add_definitions(-std=c++14)

