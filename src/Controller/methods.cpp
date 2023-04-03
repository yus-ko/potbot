#include<potbot/Controller.h>

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
    if(PUBLISH_COMMAND) __publishcmd();
}

void ControllerClass::controller()
{
    double distance = get_Distance(robot_.pose.pose.position, goal_.pose.position);
    double angle = get_Yaw(goal_.pose.orientation) - get_Yaw(robot_.pose.pose.orientation);
    if (distance < 0.05 && abs(angle) < 0.01)
    {
        geometry_msgs::Twist cmd;
        cmd_=cmd;
    }
    else if (distance < 0.3)
    {
        __PoseAlignment();
    }
    else
    {
        if (__PathCollision())
        {
            __publish_path_request();
        }
        __LineFollowing();
        
    }
}

//pure pursuite法
void ControllerClass::__LineFollowing()
{
    int robot_path_size = robot_path_.poses.size();
    if (robot_path_size == 0)
    {
        __publish_path_request();
        return;
    }

    double procces = double(robot_path_index_)/double(robot_path_size);
    ROS_INFO("line following processing: %3.1f %% index:%d/%d Done", robot_path_index_, robot_path_size, procces*100);
    if (procces > 0.8)
    {
        __publish_path_request();
    }

    double margin = PATH_TRACKING_MARGIN;
    //if (robot_path_index >= robot_path_size - 1) margin = 0.1;

    // tf2_ros::TransformListener tfListener(tf_buffer_);
    // geometry_msgs::TransformStamped transformStamped;
    // try{
    //     transformStamped = tf_buffer_.lookupTransform("robot", robot_.header.stamp, "robot", line_following_start__start_, "map");
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

    // robot_.pose.pose.position.x -= line_following_start__start_.pose.pose.position.x;
    // robot_.pose.pose.position.y -= line_following_start__start_.pose.pose.position.y;
    // robot_.pose.pose.position.z -= line_following_start__start_.pose.pose.position.z;
    // robot_.pose.pose.orientation.x -= line_following_start__start_.pose.pose.orientation.x;
    // robot_.pose.pose.orientation.y -= line_following_start__start_.pose.pose.orientation.y;
    // robot_.pose.pose.orientation.z -= line_following_start__start_.pose.pose.orientation.z;
    // robot_.pose.pose.orientation.w -= line_following_start__start_.pose.pose.orientation.w;

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
            if (l_d > 1)
            {
                __publish_path_request();
            }
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

void ControllerClass::__PoseAlignment()
{
    double v=0,omega=0;
    double yaw_target = atan2(goal_.pose.position.y - robot_.pose.pose.position.y ,goal_.pose.position.x - robot_.pose.pose.position.x);
    double yaw_now = get_Yaw(robot_.pose.pose.orientation);
    double yaw_err = yaw_target - yaw_now;
    if (get_Distance(robot_.pose.pose.position, goal_.pose.position) < 0.05)
    {
        yaw_target = get_Yaw(goal_.pose.orientation);
        yaw_err = yaw_target - yaw_now;
        if (abs(yaw_err) > 0.01)
        {
            omega = yaw_err;
        }
    }
    else
    {
        v = 0.05;
        omega = yaw_err;
    }
    cmd_.linear.x = v;
    cmd_.angular.z = omega;
}

bool ControllerClass::__PathCollision()
{
    int path_size = robot_path_.poses.size();

    // nav_msgs::MapMetaData mapinfo = local_map_.info;
    // mapinfo.origin.position.x += robot_.pose.pose.position.x;
    // mapinfo.origin.position.y += robot_.pose.pose.position.y;
    // for (int p = 0; p < path_size; p++)
    // {
    //     double px = robot_path_.poses[p].pose.position.x;
    //     double py = robot_path_.poses[p].pose.position.y;
    //     int map_index = get_index(px,py,mapinfo);
    //     // ROS_INFO("%d, %f, %f",map_index, px, py);
    //     if (map_index && local_map_.data[map_index])
    //     {
    //         geometry_msgs::Point obs = get_coordinate(map_index,local_map_.info);
    //         obs.x += robot_.pose.pose.position.x;
    //         obs.y += robot_.pose.pose.position.y;
    //         geometry_msgs::Pose pose;
    //         pose.position = obs;
    //         std::cout<<"obs";
    //         print_Pose(pose);
    //         std::cout<<"path";
    //         print_Pose(robot_path_.poses[p].pose);
    //         return true;
    //     }
    // }
    
    int map_size = local_map_.data.size();

    std::vector<geometry_msgs::Point> obs;
    double rx = robot_.pose.pose.position.x;
    double ry = robot_.pose.pose.position.y;
    double yaw = get_Yaw(robot_.pose.pose.orientation);
    // for (int i=0; i < map_size; i++)
    // {
    //     if(local_map_.data[i])
    //     {
    //         geometry_msgs::Point o = get_coordinate(i,local_map_.info);
    //         o.x += rx;
    //         o.y += ry;
    //         obs.push_back(o);
    //     }
    // }
    int scan_size = scan_.ranges.size();
    for (int i = 0; i < scan_size; i++)
    {
        if (!isinf(scan_.ranges[i]) && !isnan(scan_.ranges[i]))
        {
            double angle = i * scan_.angle_increment + scan_.angle_min + yaw;
            double distance = scan_.ranges[i] + scan_.range_min;
            double x = distance * cos(angle) + rx;
            double y = distance * sin(angle) + ry;
            geometry_msgs::Point o;
            o.x=x;
            o.y=y;
            obs.push_back(o);
        }
    }

    
    int obs_size = obs.size();
    for (int i=0; i < obs_size; i++)
    {
        
        for (int p = robot_path_index_; p < path_size; p++)
        {
            if (get_Distance(obs[i],robot_path_.poses[p].pose.position) < 0.1)
            {
                geometry_msgs::Pose pose;
                pose.position = obs[i];
                std::cout<<"obs";
                print_Pose(pose);
                std::cout<<"path";
                print_Pose(robot_path_.poses[p].pose);
                return true;
            }
        }
    }

    return false;
    
}

void ControllerClass::__publishcmd()
{
    pub_cmd_.publish(cmd_);
}

void ControllerClass::__publish_path_request()
{
    std_msgs::Empty empty;
    pub_path_request_.publish(empty);
}