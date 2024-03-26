cmake_minimum_required(VERSION 3.0.2)
project(potbot_pcl)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  potbot_lib
  potbot_msgs
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} potbot_lib
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

add_executable(         ${PROJECT_NAME}_cl3d src/constructor.cpp src/debug_main.cpp src/methods.cpp src/property.cpp src/callback.cpp)
target_link_libraries(  ${PROJECT_NAME}_cl3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})



add_definitions(-std=c++14)