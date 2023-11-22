# potbot

# Installation
 
はじめに以下のコマンドで必要なパッケージをインストールしてください
 
```bash
sudo apt install ros-melodic-gazebo-ros
sudo apt install ros-melodic-gazebo-ros-control 
sudo apt install ros-melodic-ros-control
sudo apt install ros-melodic-ros-controllers 
```
```bash
cd ~/catkin_ws/src
git clone https://github.com/vstoneofficial/megarover_samples
git clone https://github.com/ROBOTIS-GIT/turtlebot3
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
