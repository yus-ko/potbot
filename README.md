# 複数のパッケージで構成

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
