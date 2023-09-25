#include<potbot/Controller.h>

void ControllerClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
}

void ControllerClass::path_callback(const nav_msgs::Path& msg)
{
    if (robot_path_.header.stamp != msg.header.stamp)
    {
        nav_msgs::Path init;
        robot_path_ = init;
        robot_path_.header = msg.header;
        static tf2_ros::TransformListener tfListener(tf_buffer_);
        for (int i = 0; i < msg.poses.size(); i++)
        {
            geometry_msgs::TransformStamped transform;
            geometry_msgs::PoseStamped target_point;
            //target_point.header.frame_id = "map";
            try 
            {
                // ロボット座標系の経路を世界座標系に変換
                transform = tf_buffer_.lookupTransform("map", msg.header.frame_id, ros::Time());
                tf2::doTransform(msg.poses[i], target_point, transform);
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_ERROR("TF Ereor in ControllerClass::path_callback: %s", ex.what());
                break;
            }
            robot_path_.poses.push_back(target_point);
        }
        done_init_pose_alignment_ = false;
        robot_path_index_ = 0;
        line_following_start_ = robot_;
    }

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

void ControllerClass::__segment_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacle_segment_ = msg;
}

void ControllerClass::__param_callback(const potbot::ControllerConfig& param, uint32_t level)
{
    publish_command_ = param.publish_control_command;
}