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

void ControllerClass::mainloop()
{
    ros::Rate loop_rate(50);
	while (ros::ok())
	{
        
        manage();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ControllerClass::manage()
{
    // geometry_msgs::PoseStamped robot_pose;
    // while (!get_WorldCoordinate("robot", odom_.header.stamp, robot_pose, tf_buffer_)){}
    // robot_.header = robot_pose.header;
    // robot_.pose.pose = robot_pose.pose;
    robot_ = odom_;
    // print_Pose(robot_.pose.pose);

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

    ROS_INFO("line following: %d/%d", robot_path_index_, robot_path_size);

    double margin = PATH_TRACKING_MARGIN;
    //if (robot_path_index >= robot_path_size - 1) margin = 0.1;

    // tf2_ros::TransformListener tfListener(tf_buffer_);
    // geometry_msgs::TransformStamped transformStamped;
    // try{
    //     transformStamped = tf_buffer_.lookupTransform("robot", robot_.header.stamp, "robot", line_following_start_, "map");
    // }
    // catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    //     return;
    // }
    // robot_.header = transformStamped.header;
    // robot_.pose.pose.position.x = transformStamped.transform.translation.x;
    // robot_.pose.pose.position.y = transformStamped.transform.translation.y;
    // robot_.pose.pose.position.z = transformStamped.transform.translation.z;
    // robot_.pose.pose.orientation.x = transformStamped.transform.rotation.x;
    // robot_.pose.pose.orientation.y = transformStamped.transform.rotation.y;
    // robot_.pose.pose.orientation.z = transformStamped.transform.rotation.z;
    // robot_.pose.pose.orientation.w = transformStamped.transform.rotation.w;

    // robot_.pose.pose.position.x -= line_following_start_.pose.pose.position.x;
    // robot_.pose.pose.position.y -= line_following_start_.pose.pose.position.y;
    // robot_.pose.pose.position.z -= line_following_start_.pose.pose.position.z;
    // robot_.pose.pose.orientation.x -= line_following_start_.pose.pose.orientation.x;
    // robot_.pose.pose.orientation.y -= line_following_start_.pose.pose.orientation.y;
    // robot_.pose.pose.orientation.z -= line_following_start_.pose.pose.orientation.z;
    // robot_.pose.pose.orientation.w -= line_following_start_.pose.pose.orientation.w;

    print_Pose(robot_.pose.pose);

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