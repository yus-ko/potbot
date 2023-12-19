# potbot

# Installation
 
はじめに以下のコマンドで必要なパッケージをインストールしてください
 
```bash
sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt install ros-melodic-gmapping ros-melodic-amcl ros-melodic-map-server ros-melodic-robot-localization
mkdir ~/.gazebo/model
cd ~/.gazebo/model
git clone https://github.com/osrf/gazebo_models
```
```bash
cd ~/catkin_ws/src
git clone https://github.com/vstoneofficial/megarover_samples
git clone https://github.com/ROBOTIS-GIT/turtlebot3
# git clone -b melodic https://github.com/ros-perception/image_pipeline
git clone https://github.com/yus-ko/multiple_robots_slam
```

# 起動方法

**Localization.launch**
- オドメトリ計算とセンサデータ処理(LRFのクラスタリング)

**Controller.launch**
- ライン追従のための制御指令計算

**PathPlanning.launch**
- 人工ポテンシャル法による経路計画

**Filter.launch**
- アンセンテッドカルマンフィルタによる障害物に対する速度予測
  
**robot_0.launch**
- 上記をまとめて起動
