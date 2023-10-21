// #include <iostream>
#include <ros/ros.h>
// #include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <potbot/beego_encoder.h>

namespace potbot_lib{

    const int SUCCESS = 1;
    const int FAIL = 0;

    const int MEGAROVER = 0;
    const int TURTLEBOT3 = 1;
    const int BEEGO = 2;

    const int DEAD_RECKONING = 0;
    const int PARTICLE_FILTER = 1;

    const int CSV_PATH = 0;
    const int POTENTIAL_METHOD = 1;

    namespace utility{
        void getRPY(geometry_msgs::Quaternion orientation, double &roll, double &pitch, double &yaw);
        void getQuat(double roll, double pitch, double yaw, geometry_msgs::Quaternion &orientation);
        geometry_msgs::Quaternion get_Quat(double roll, double pitch, double yaw);
        double get_Yaw(geometry_msgs::Quaternion orientation);
        double get_Distance(geometry_msgs::Point position1, geometry_msgs::Point position2);
        void print_Pose(geometry_msgs::Pose pose);
        int get_tf(geometry_msgs::PoseStamped pose_in, geometry_msgs::PoseStamped &pose_out, tf2_ros::Buffer &buffer);
        int get_WorldCoordinate(std::string target_frame, ros::Time time, geometry_msgs::PoseStamped &Wcood, tf2_ros::Buffer &buffer);
        geometry_msgs::Point get_coordinate(int index, nav_msgs::MapMetaData info);
        int get_index(double x, double y, nav_msgs::MapMetaData info);
    }
}



