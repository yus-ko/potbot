cmake_minimum_required(VERSION 3.0.2)
project(potbot)

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
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PathPlanning.cfg
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
  src/StereoCamera/classification/constructor.cpp
  src/StereoCamera/classification/methods.cpp
  src/StereoCamera/classification/debug_main.cpp
  src/StereoCamera/classification/debug_methods.cpp
  src/StereoCamera/classification/property.cpp
)

add_executable(${PROJECT_NAME}_sc # stereo camera 
  src/StereoCamera/convCamData/constructor.cpp
  src/StereoCamera/convCamData/methods.cpp
  src/StereoCamera/convCamData/debug_main.cpp
  src/StereoCamera/convCamData/property.cpp
)

add_executable(${PROJECT_NAME}_mv # measure Velocity
 src/StereoCamera/measureVelocity/constructor.cpp
 src/StereoCamera/measureVelocity/methods.cpp
 src/StereoCamera/measureVelocity/debug_main.cpp
 src/StereoCamera/measureVelocity/debug_methods.cpp
 src/StereoCamera/measureVelocity/property.cpp
)

add_executable(${PROJECT_NAME}_si # point cloud
  src/StereoCamera/synchroImage/constructor.cpp
  src/StereoCamera/synchroImage/methods.cpp
  src/StereoCamera/synchroImage/debug_main.cpp
  src/StereoCamera/synchroImage/debug_methods.cpp
  src/StereoCamera/synchroImage/property.cpp
)
add_executable(${PROJECT_NAME}_frm # manage
  src/StereoCamera/frameRateManagement/constructor.cpp
  src/StereoCamera/frameRateManagement/methods.cpp
  src/StereoCamera/frameRateManagement/debug_main.cpp
  src/StereoCamera/frameRateManagement/debug_methods.cpp
  src/StereoCamera/frameRateManagement/property.cpp
)

add_executable(${PROJECT_NAME}_im # image matching
   src/StereoCamera/imageMatching/constructor.cpp
   src/StereoCamera/imageMatching/methods.cpp
   src/StereoCamera/imageMatching/debug_main.cpp
   src/StereoCamera/imageMatching/debug_methods.cpp
   src/StereoCamera/imageMatching/property.cpp
 )

add_executable(${PROJECT_NAME}_estimate # manage
  src/StereoCamera/estimationVelocity/constructor.cpp
  src/StereoCamera/estimationVelocity/methods.cpp
  src/StereoCamera/estimationVelocity/debug_main.cpp
  src/StereoCamera/estimationVelocity/property.cpp
  src/StereoCamera/estimationVelocity/debug_methods.cpp
)

add_executable(${PROJECT_NAME}_pp
  src/PathPlanning/callback.cpp
  src/PathPlanning/constructor.cpp
  src/PathPlanning/methods.cpp
  src/PathPlanning/debug_main.cpp
  src/PathPlanning/property.cpp
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
  src/Localization/callback.cpp
  src/Localization/constructor.cpp
  src/Localization/methods.cpp
  src/Localization/debug_main.cpp
  src/Localization/property.cpp
)

add_executable(${PROJECT_NAME}_co
  src/Controller/constructor.cpp
  src/Controller/methods.cpp
  src/Controller/debug_main.cpp
  src/Controller/property.cpp
)

add_dependencies(${PROJECT_NAME}_cc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_sc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_mv ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_si ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_frm ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_im ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_estimate ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_pp ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_te ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_rv ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_lo ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_co ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${PROJECT_NAME}_pp ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_cc ${catkin_LIBRARIES}) # classification
target_link_libraries(${PROJECT_NAME}_sc ${catkin_LIBRARIES}) # stereo camera
target_link_libraries(${PROJECT_NAME}_mv ${catkin_LIBRARIES} ) # measurement velocity 
target_link_libraries(${PROJECT_NAME}_si ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # synchro image
target_link_libraries(${PROJECT_NAME}_frm ${catkin_LIBRARIES} ) # manege 
target_link_libraries(${PROJECT_NAME}_im ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # image matching
target_link_libraries(${PROJECT_NAME}_estimate ${catkin_LIBRARIES} ) # estimate 
target_link_libraries(${PROJECT_NAME}_pp ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_te ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_rv ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_lo ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_co ${catkin_LIBRARIES} )

add_definitions(-std=c++11)
