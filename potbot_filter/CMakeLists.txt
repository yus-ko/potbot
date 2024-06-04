cmake_minimum_required(VERSION 3.0.2)
project(potbot_filter)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_ros
  potbot_lib
  potbot_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Clustering.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_estimation 2dscan_clustering
  # CATKIN_DEPENDS 
  # DEPENDS eigen opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(state_estimation src/state_estimation.cpp)
target_link_libraries(state_estimation ${catkin_LIBRARIES})

add_executable(state_estimation_node src/state_estimation_node.cpp)
target_link_libraries(state_estimation_node state_estimation)

add_library(2dscan_clustering src/2dscan_clustering.cpp)
add_dependencies(2dscan_clustering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(2dscan_clustering ${catkin_LIBRARIES})

add_executable(2dscan_clustering_node src/2dscan_clustering_node.cpp)
target_link_libraries(2dscan_clustering_node 2dscan_clustering)

add_definitions(-std=c++14)


