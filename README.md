# potbot

# Installation
 
はじめに以下のコマンドで必要なパッケージをインストールしてください
 
```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers
sudo apt install ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-map-server ros-$ROS_DISTRO-robot-localization
mkdir ~/.gazebo/model
cd ~/.gazebo/model
git clone https://github.com/osrf/gazebo_models
```
```bash
cd ~/catkin_ws/src
git clone https://github.com/vstoneofficial/megarover_samples
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/ROBOTIS-GIT/turtlebot3
git clone https://github.com/yus-ko/multiple_robots_slam
git clone https://github.com/yus-ko/potbot
cd ..
```
```bash
catkin build megarover_samples turtlebot3_msgs turtlebot3 multiple_robots_slam potbot
```

# 起動方法

**turtlebot3_with_willowgarage.launch**
gazebo環境
  
**demo.launch**
potbot起動
