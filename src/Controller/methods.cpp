#include<potbot/Controller.h>

void ControllerClass::odom_callback(const nav_msgs::Odometry& msg)
{
    //encoder_value.header = msg.header;
    robot_ = msg;
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

void ControllerClass::path_callback(const nav_msgs::Path& msg)
{
    robot_path_ = msg;
    robot_path_index_ = 0;
}

void ControllerClass::manage()
{
    controller();
    if(PUBLISH_COMMAND) publishcmd();
}

void ControllerClass::controller()
{
    line_following();
}

//pure pursuite法
void ControllerClass::line_following()
{
    int robot_path_size = robot_path_.poses.size();
    if (robot_path_size == 0) return;

    std::cout<< "robot_path_size = " << robot_path_size <<std::endl;
    std::cout<< "robot_path_index = " << robot_path_index_ <<std::endl;

    double margin = PATH_TRACKING_MARGIN;
    //if (robot_path_index >= robot_path_size - 1) margin = 0.1;

    geometry_msgs::Point sub_goal;
    double l_d;
    while(true)
    {
        sub_goal = robot_path_.poses[robot_path_index_].pose.position;
        l_d = sqrt(pow(robot_.pose.pose.position.x - sub_goal.x,2) + pow(robot_.pose.pose.position.y - sub_goal.y,2));
        if (l_d <= margin)// || (sub_goal.x - odom.pose.pose.position.x < AHEAD_PATH))
        //if (sqrt(pow(odom.pose.pose.position.x - sub_goal.x,2)) <= 0.05)
        {
            robot_path_index_++;
            if (robot_path_index_ >= robot_path_size-2)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    
    geometry_msgs::Twist cmd;

    if (robot_path_index_ < robot_path_size)
    {   
        double roll, pitch, yaw;
        tf2::Quaternion quat;
        tf2::convert(robot_.pose.pose.orientation, quat);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        double alpha = atan2(sub_goal.y - robot_.pose.pose.position.y, sub_goal.x - robot_.pose.pose.position.x) - yaw;

        cmd.linear.x = 0.2;
        cmd.angular.z = 2*cmd.linear.x*sin(alpha)/l_d;
        if(abs(cmd.angular.z) > M_PI_2)
        {
            //cmd.linear.x = 0.0;
        }
    
    }
    cmd_ = cmd;
}

void ControllerClass::publishcmd()
{
    pub_cmd_.publish(cmd_);
}