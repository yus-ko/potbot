cmake_minimum_required(VERSION 3.0.2)
project(potbot_controller)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_ros
  potbot_lib
  potbot_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
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
)

# add_library(${PROJECT_NAME} src/Controller.cpp)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# add_executable(${PROJECT_NAME}_node src/node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_definitions(-std=c++14)

