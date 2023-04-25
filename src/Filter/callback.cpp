#include<potbot/Controller.h>

void ControllerClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
}

void ControllerClass::path_callback(const nav_msgs::Path& msg)
{
    robot_path_ = msg;
    robot_path_index_ = 0;
    line_following_start_ = robot_;

}

void ControllerClass::__goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal");
    __publish_path_request();
}

void ControllerClass::__local_map_callback(const nav_msgs::OccupancyGrid& msg)
{
    local_map_ = msg;
}

void ControllerClass::__scan_callback(const sensor_msgs::LaserScan& msg)
{
    scan_ = msg;
}