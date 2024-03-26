#include<potbot_controller/Controller.h>

void ControllerClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    nav_msgs::Odometry odom = msg;
    geometry_msgs::TransformStamped transform;
    geometry_msgs::Pose target_point;
    try 
    {
        // odom座標系の自己位置をmap座標系に変換
        transform = tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, odom.header.frame_id, odom.header.stamp);
        // transform = tf_buffer_.lookupTransform(FRAME_ID_ROBOT_BASE, odom.header.frame_id, odom.header.stamp);
        tf2::doTransform(odom.pose.pose, target_point, transform);
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_ERROR("TF Ereor in ControllerClass::__odom_callback: %s", ex.what());
        return;
    }

    odom_ = odom_;
    odom_.header.frame_id = FRAME_ID_GLOBAL;
    odom_.pose.pose = target_point;

    manage();
}

void ControllerClass::__path_callback(const nav_msgs::Path& msg)
{
    //ROS_INFO("path_callback");
    //ROS_INFO("%f",msg.header.stamp.toSec());
    if (true || robot_path_.header.stamp != msg.header.stamp)
    {
        
        if (msg.header.frame_id != FRAME_ID_GLOBAL)
        {
            nav_msgs::Path init;
            robot_path_ = init;
            robot_path_.header = msg.header;
            for (int i = 0; i < msg.poses.size(); i++)
            {
                geometry_msgs::TransformStamped transform;
                geometry_msgs::PoseStamped target_point;
                //target_point.header.frame_id = FRAME_ID_GLOBAL;
                try 
                {
                    // ロボット座標系の経路を世界座標系に変換
                    transform = tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, msg.header.frame_id, msg.header.stamp);
                    tf2::doTransform(msg.poses[i], target_point, transform);
                }
                catch (tf2::TransformException &ex) 
                {
                    ROS_ERROR("TF Ereor in ControllerClass::path_callback: %s", ex.what());
                    break;
                }
                robot_path_.poses.push_back(target_point);
            }
            
        }
        else
        {
            robot_path_ = msg;
        }
        done_init_pose_alignment_ = false;
        robot_path_index_ = 0;
        line_following_start_ = robot_;
    }

}

void ControllerClass::__goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal: controller");
    __publish_path_request();
}

void ControllerClass::__param_callback(const potbot_msgs::ControllerConfig& param, uint32_t level)
{
    PUBLISH_COMMAND             = param.publish_control_command;
    PATH_TRACKING_MARGIN        = param.pure_pursuit_distance_to_lookahead_point;
}