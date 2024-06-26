cmake_minimum_required(VERSION 3.0.2)
project(potbot_pcl)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_ros
  potbot_lib
  potbot_msgs
)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ClusteringParam.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_cl3d
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(         ${PROJECT_NAME}_main src/main_2.cpp)
target_link_libraries(  ${PROJECT_NAME}_main ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(         ${PROJECT_NAME}_tutorial src/tutorial.cpp)
target_link_libraries(  ${PROJECT_NAME}_tutorial ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(         ${PROJECT_NAME}_tutorial2 src/main.cpp)
#target_link_libraries(  ${PROJECT_NAME}_tutorial2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(${PROJECT_NAME}_cl3d src/pcl_method.cpp)
add_dependencies(${PROJECT_NAME}_cl3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_cl3d ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_cl3d_node src/pcl_node.cpp)
target_link_libraries(${PROJECT_NAME}_cl3d_node ${PROJECT_NAME}_cl3d)

add_definitions(-std=c++14)