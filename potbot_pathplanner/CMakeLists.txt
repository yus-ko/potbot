cmake_minimum_required(VERSION 3.0.2)
project(potbot_pathplanner)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  potbot_lib
  potbot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} potbot_lib
  # CATKIN_DEPENDS 
  # DEPENDS eigen opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_pp
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_pp ${catkin_LIBRARIES})

add_definitions(-std=c++14)