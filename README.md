# potbot

# Installation
 
はじめに以下のコマンドで必要なパッケージをインストールしてください
 
```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers
sudo apt install ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-map-server ros-$ROS_DISTRO-robot-localization
mkdir -p ~/.gazebo/model
cd ~/.gazebo/model
git clone https://github.com/osrf/gazebo_models
```
```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/ROBOTIS-GIT/turtlebot3
git clone https://github.com/yus-ko/potbot
```
```bash
cd ~/catkin_ws
catkin build turtlebot3 potbot
```

# 起動方法

```bash
roslaunch potbot turtlebot3_with_willowgarage.launch
```
gazebo環境

<<<<<<< HEAD
**Filter.launch**
- アンセンテッドカルマンフィルタによる障害物に対する速度予測
  
**robot_0.launch**
- 上記をまとめて起動

**multi_robot.launch**
- gazebo

**potbot_megarover.launch**
- ナビゲーション
=======
```bash
roslaunch potbot demo.launch
```
ナビゲーションプログラムの起動
>>>>>>> initbuild
