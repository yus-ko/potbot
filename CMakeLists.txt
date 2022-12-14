cmake_minimum_required(VERSION 3.0.2)
project(autonomous_mobile_robot_2022)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
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

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autonomous_mobile_robot_2022
    CATKIN_DEPENDS roscpp sensor_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/autonomous_mobile_robot_2022.cpp
# )

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

add_executable(${PROJECT_NAME}_pc # manage
  src/PointCloud/constructor.cpp
  src/PointCloud/methods.cpp
  src/PointCloud/debug_main.cpp
  src/PointCloud/property.cpp
)

add_executable(${PROJECT_NAME}_rc # manage
  src/RobotControl/constructor.cpp
  src/RobotControl/methods.cpp
  src/RobotControl/debug_main.cpp
  src/RobotControl/property.cpp
)

add_executable(${PROJECT_NAME}_pm # manage
  src/PotentialMethod/constructor.cpp
  src/PotentialMethod/methods.cpp
  src/PotentialMethod/debug_main.cpp
  src/PotentialMethod/property.cpp
)

add_executable(${PROJECT_NAME}_te # manage
  src/test/test.cpp
  #src/test/spline.cpp
  #src/test/motor_response.cpp
  #src/test/localization_accuracy.cpp
)

add_executable(${PROJECT_NAME}_rv # manage
  src/rviz/constructor.cpp
  src/rviz/methods.cpp
  src/rviz/debug_main.cpp
  src/rviz/property.cpp
)

add_executable(${PROJECT_NAME}_tf # manage
  src/tf/constructor.cpp
  src/tf/methods.cpp
  src/tf/debug_main.cpp
  src/tf/property.cpp
)

add_dependencies(${PROJECT_NAME}_cc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_sc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_mv ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_si ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_frm ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_im ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_estimate ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_pc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_rc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_pm ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_te ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_rv ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_tf ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_cc ${catkin_LIBRARIES}) # classification
target_link_libraries(${PROJECT_NAME}_sc ${catkin_LIBRARIES}) # stereo camera
target_link_libraries(${PROJECT_NAME}_mv ${catkin_LIBRARIES} ) # measurement velocity 
target_link_libraries(${PROJECT_NAME}_si ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # synchro image
target_link_libraries(${PROJECT_NAME}_frm ${catkin_LIBRARIES} ) # manege 
target_link_libraries(${PROJECT_NAME}_im ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # image matching
target_link_libraries(${PROJECT_NAME}_estimate ${catkin_LIBRARIES} ) # estimate 
target_link_libraries(${PROJECT_NAME}_pc ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_rc ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_pm ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_te ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_rv ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_tf ${catkin_LIBRARIES} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autonomous_mobile_robot_2022.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_definitions(-std=c++11)
