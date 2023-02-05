cmake_minimum_required(VERSION 3.0.2)
project(autonomous_mobile_robot_2022)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  cv_bridge
  pcl_ros
  dynamic_reconfigure
  #beego_control
  #dynamixel_sdk
  #dynamixel_sdk_examples
  #obstacle_detection_2019
  visualization_msgs
  #autonomous_mobile_robot
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
  ClassificationData.msg
  ClassificationElement.msg
  ClassificationVelocityData.msg
  synchronizedImage.msg
  MaskImageData.msg
  ImageMatchingData.msg
  ImageFlow2D.msg
  SensorMapData.msg
  #BridgeImageData.msg
  PointCloudData.msg
  ClusterData.msg
  PotentialValue.msg
  PathPlan.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_cc # classification
  src/classification/constructor.cpp
  src/classification/methods.cpp
  src/classification/debug_main.cpp
  src/classification/debug_methods.cpp
  src/classification/property.cpp
)

add_executable(${PROJECT_NAME}_sc # stereo camera 
  src/convCamData/constructor.cpp
  src/convCamData/methods.cpp
  src/convCamData/debug_main.cpp
  src/convCamData/property.cpp
)

add_executable(${PROJECT_NAME}_mv # measure Velocity
 src/measureVelocity/constructor.cpp
 src/measureVelocity/methods.cpp
 src/measureVelocity/debug_main.cpp
 src/measureVelocity/debug_methods.cpp
 src/measureVelocity/property.cpp
)

add_executable(${PROJECT_NAME}_si # point cloud
  src/synchroImage/constructor.cpp
  src/synchroImage/methods.cpp
  src/synchroImage/debug_main.cpp
  src/synchroImage/debug_methods.cpp
  src/synchroImage/property.cpp
)
add_executable(${PROJECT_NAME}_frm # manage
  src/frameRateManagement/constructor.cpp
  src/frameRateManagement/methods.cpp
  src/frameRateManagement/debug_main.cpp
  src/frameRateManagement/debug_methods.cpp
  src/frameRateManagement/property.cpp
)

add_executable(${PROJECT_NAME}_im # image matching
   src/imageMatching/constructor.cpp
   src/imageMatching/methods.cpp
   src/imageMatching/debug_main.cpp
   src/imageMatching/debug_methods.cpp
   src/imageMatching/property.cpp
 )

add_executable(${PROJECT_NAME}_estimate # manage
  src/estimationVelocity/constructor.cpp
  src/estimationVelocity/methods.cpp
  src/estimationVelocity/debug_main.cpp
  src/estimationVelocity/property.cpp
  src/estimationVelocity/debug_methods.cpp
)

add_executable(${PROJECT_NAME}_pm
  src/PotentialMethod/constructor.cpp
  src/PotentialMethod/methods.cpp
  src/PotentialMethod/debug_main.cpp
  src/PotentialMethod/property.cpp
)

add_executable(${PROJECT_NAME}_te
  src/test/test.cpp
  #src/test/spline.cpp
  #src/test/motor_response.cpp
  #src/test/localization_accuracy.cpp
)

add_executable(${PROJECT_NAME}_rv
  src/rviz/constructor.cpp
  src/rviz/methods.cpp
  src/rviz/debug_main.cpp
  src/rviz/property.cpp
)

add_executable(${PROJECT_NAME}_lo
  src/Localization/constructor.cpp
  src/Localization/methods.cpp
  src/Localization/debug_main.cpp
  src/Localization/property.cpp
)

add_dependencies(${PROJECT_NAME}_cc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_sc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_mv ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_si ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_frm ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_im ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_estimate ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_pm ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_te ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_rv ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_lo ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_cc ${catkin_LIBRARIES}) # classification
target_link_libraries(${PROJECT_NAME}_sc ${catkin_LIBRARIES}) # stereo camera
target_link_libraries(${PROJECT_NAME}_mv ${catkin_LIBRARIES} ) # measurement velocity 
target_link_libraries(${PROJECT_NAME}_si ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # synchro image
target_link_libraries(${PROJECT_NAME}_frm ${catkin_LIBRARIES} ) # manege 
target_link_libraries(${PROJECT_NAME}_im ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # image matching
target_link_libraries(${PROJECT_NAME}_estimate ${catkin_LIBRARIES} ) # estimate 
target_link_libraries(${PROJECT_NAME}_pm ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_te ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_rv ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_lo ${catkin_LIBRARIES} )

add_definitions(-std=c++11)
