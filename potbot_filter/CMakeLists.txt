cmake_minimum_required(VERSION 3.0.2)
project(potbot_filter)

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

add_executable(state_estimation
  src/constructor.cpp
  src/methods.cpp
  src/debug_main.cpp
  src/callback.cpp
  src/property.cpp
)
target_link_libraries(state_estimation ${catkin_LIBRARIES})

add_executable(2dscan_clustering src/2dscan_clustering.cpp)
target_link_libraries(2dscan_clustering ${catkin_LIBRARIES})

add_definitions(-std=c++14)


