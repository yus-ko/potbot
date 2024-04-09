# potbot

# Installation
 
はじめに以下のコマンドで必要なパッケージをインストールしてください
 
```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers
sudo apt install ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-map-server ros-$ROS_DISTRO-robot-localization
mkdir -p ~/.gazebo/models & cd ~/.gazebo/models
git clone https://github.com/osrf/gazebo_models
mv ~/.gazebo/models/gazebo_models/* ~/.gazebo/models/ & sudo rm -r ~/.gazebo/models/gazebo_models/
```
```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/ROBOTIS-GIT/turtlebot3
git clone https://github.com/yus-ko/potbot_core
git clone https://github.com/yus-ko/potbot
```
```bash
cd ~/catkin_ws
catkin build turtlebot3 potbot
```

# 起動方法

gazebo環境
```bash
roslaunch potbot turtlebot3_with_willowgarage.launch
```

ナビゲーションプログラムの起動
```bash
roslaunch potbot demo.launch
```