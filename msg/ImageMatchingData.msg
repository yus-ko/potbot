#image matching data
#
#ROSヘッダ
std_msgs/Header header
#マップサイズ
#実測サイズ
std_msgs/Float32 width
std_msgs/Float32 height
std_msgs/Float32 res
#ピクセルサイズ
std_msgs/Int32 widthInt
std_msgs/Int32 heightInt
#マップの中心点（センサ座標）
geometry_msgs/Point cp
#インデックスデータ
std_msgs/Int32[] index
#マッチングデータ
autonomous_mobile_robot_2022/ImageFlow2D[] data