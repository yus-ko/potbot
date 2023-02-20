#include<potbot/Controller.h>

void ControllerClass::odom_callback(const nav_msgs::Odometry& msg)
{
    //encoder_value.header = msg.header;

    double roll, pitch, yaw;
    tf2::Quaternion quat;
    tf2::convert(msg.pose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    robot_pose_x_ = msg.pose.pose.position.x;
    robot_pose_y_ = msg.pose.pose.position.y;
    robot_pose_theta_ = yaw;
    ROS_INFO("x: %f, y: %f, theta: %f", robot_pose_x_, robot_pose_y_, robot_pose_theta_/M_PI*180);
    manage();
}

void ControllerClass::manage()
{
    controller();
    publishcmd();
}

void ControllerClass::controller()
{
    // bool straight = false;
    // bool rotation = false;

    // int size = robot_path_.size();
    // double distance ,angle;
    // while (robot_path_index_ < size)
    // {
    //     double x_diff = robot_path_[robot_path_index_].x - robot_pose_x_;
    //     double y_diff = robot_path_[robot_path_index_].y - robot_pose_y_;
    //     distance = sqrt(pow(x_diff,2) + pow(y_diff,2));
    //     angle = atan2(x_diff, y_diff) - robot_pose_theta_;
    //     if (abs(angle) < 0.3490)
    //     {
    //         rotation = false;
    //         if (distance < 0.5)
    //         {
    //             straight = false;
    //             robot_path_index_++;
    //             ROS_INFO("--------------------------------------%d", robot_path_index_);
    //         }
    //         else
    //         {
    //             straight = true;
    //             break;
    //         }
    //     }
    //     else
    //     {
    //         rotation = true;
    //         break;
    //     }
        
    // }
    // geometry_msgs::Twist cmd;
    // if (robot_path_index_ < size)
    // {
    //     ROS_INFO("distance: %f, angle: %f, path: %d", distance, angle/M_PI*180, robot_path_index_);

        
    //     ROS_INFO("straight %d: rotation %d", straight,rotation);
    //     if (straight)
    //         cmd.linear.x = 0.3;
    //     if (rotation)
    //         cmd.angular.z = angle;
    // }
    
    // cmd_ = cmd;

    geometry_msgs::Twist cmd;
    int size = robot_path_.size();
    while (robot_path_index_ < size)
    {
        double x_diff = robot_path_[robot_path_index_].x - robot_pose_x_;
        double y_diff = robot_path_[robot_path_index_].y - robot_pose_y_;
        double distance = sqrt(pow(x_diff,2) + pow(y_diff,2));
        if (distance > 0.2)
        {
            break;
        }
        robot_path_index_++;
    }
    ROS_INFO("%d", robot_path_index_);
    if (robot_path_index_ < size)
    {
        double x_diff = robot_path_[robot_path_index_].x - robot_pose_x_;
        double y_diff = robot_path_[robot_path_index_].y - robot_pose_y_;

        cmd.linear.x = 0.2;
        double alpha = atan2(y_diff, x_diff) - robot_pose_theta_;
        double ld = sqrt(pow(x_diff,2) + pow(y_diff,2));
        cmd.angular.z = 2*cmd.linear.x*sin(alpha)/ld;
    }
    cmd_ = cmd;
}
void ControllerClass::publishcmd()
{
    pub_cmd_.publish(cmd_);
}