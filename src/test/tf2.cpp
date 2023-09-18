#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf2_example_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);    //実行するのは最初の1回だけ（コンストラクタで実行するなど）


    ros::Rate rate(50.0);
    while (nh.ok()) {

        geometry_msgs::TransformStamped transform;
        try {
            // ロボット座標系を世界座標系に変換
            // 世界座標系、ロボット座標系
            transform = tfBuffer.lookupTransform("map", "my_robot", ros::Time());   //変換のたびに実行

            // 座標変換を行う例
            geometry_msgs::PointStamped source_point;
            geometry_msgs::PointStamped target_point;

            // source_point.header.frame_id = "base_footprint";
            // source_point.header.stamp = ros::Time();
            source_point.point.x = 2;
            source_point.point.y = 0;
            source_point.point.z = 0;

            tf2::doTransform(source_point, target_point, transform);

            ROS_INFO("Transformed Point: (%f, %f, %f) in target_frame",
                     target_point.point.x, target_point.point.y, target_point.point.z);
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("TransformException: %s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
