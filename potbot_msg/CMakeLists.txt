cmake_minimum_required(VERSION 3.0.2)
project(potbot_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  # ClassificationData.msg
  # ClassificationElement.msg
  # ClassificationVelocityData.msg
  # synchronizedImage.msg
  # MaskImageData.msg
  # ImageMatchingData.msg
  # ImageFlow2D.msg
  # SensorMapData.msg
  #BridgeImageData.msg
  # PointCloudData.msg
  # ClusterData.msg
  PotentialValue.msg
  State.msg
  StateArray.msg
  # OdometryArray.msg
  rpc.msg
  vel.msg
  beego_encoder.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  # nav_msgs
)

# generate_dynamic_reconfigure_options(
#   cfg/PathPlanning.cfg
#   cfg/Localization.cfg
#   cfg/Controller.cfg
# )

# catkin_package(
#   CATKIN_DEPENDS
#   roscpp
#   sensor_msgs
#   message_runtime 
#   # nav_msgs
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()
add_definitions(-std=c++14)