#include<potbot_controller/Controller.h>

void ControllerClass::mainloop()
{

    ros::spin();

    // ros::Rate loop_rate(50);
	// while (ros::ok())
	// {
    //     manage();
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
}

void ControllerClass::manage()
{
    // geometry_msgs::PoseStamped robot_pose;
    // while (!get_WorldCoordinate("robot", odom_.header.stamp, robot_pose, tf_buffer_)){}
    // robot_.header = robot_pose.header;
    // robot_.pose.pose = robot_pose.pose;
    robot_ = odom_;
    // print_Pose(robot_.pose.pose);
    // ROS_INFO("path_size: %d", robot_path_.poses.size());
    //potbot_lib::utility::print_Pose(goal_.pose);
    if (!robot_path_.poses.empty() > 0) controller();
    if (PUBLISH_COMMAND) __publishcmd();
}

void ControllerClass::controller()
{
    double distance = potbot_lib::utility::get_Distance(robot_.pose.pose.position, goal_.pose.position);
    double angle = potbot_lib::utility::get_Yaw(goal_.pose.orientation) - potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation);
    if (distance < 0.05 && abs(angle) < 0.03)
    {
        geometry_msgs::Twist cmd;
        cmd_=cmd;
    }
    else if (distance < 0.3)
    {
        __PoseAlignment(goal_.pose);
    }
    else
    {
        __LineFollowing();
    }
}

//pure pursuite法
void ControllerClass::__LineFollowing()
{
    size_t robot_path_size = robot_path_.poses.size();
    double procces = double(robot_path_index_+1)/double(robot_path_size);
    //ROS_INFO("line following processing: %3.1f %% index:%d/%d Done", robot_path_index_, robot_path_size, procces*100);
    if (procces > 0.9 && potbot_lib::utility::get_Distance(robot_path_.poses[robot_path_size-1].pose.position, goal_.pose.position) > 0.3)
    {
        __publish_path_request();
    }

    double margin = PATH_TRACKING_MARGIN;

    //print_Pose(robot_.pose.pose);

    geometry_msgs::Point sub_goal;
    double l_d;
    while(true)
    {
        sub_goal = robot_path_.poses[robot_path_index_].pose.position;
        l_d = sqrt(pow(robot_.pose.pose.position.x - sub_goal.x,2) + pow(robot_.pose.pose.position.y - sub_goal.y,2));
        if (l_d <= margin)
        {
            robot_path_index_++;
            if (robot_path_index_ >= robot_path_size-1)
            {
                break;
            }
        }
        else
        {
            if (l_d > 1.0)
            {
                __publish_path_request();
            }
            else
            {
                visualization_msgs::Marker lookahead;
                lookahead.header                = robot_.header;

                lookahead.ns                    = "LookAhead";
                lookahead.id                    = 0;
                lookahead.lifetime              = ros::Duration(0);

                lookahead.type                  = visualization_msgs::Marker::SPHERE;
                lookahead.action                = visualization_msgs::Marker::MODIFY;
                
                geometry_msgs::PoseStamped pose_in;
                pose_in.header                  = robot_path_.header;
                pose_in.pose                    = robot_path_.poses[robot_path_index_].pose;
                lookahead.pose                  = potbot_lib::utility::get_tf(tf_buffer_, pose_in, FRAME_ID_ROBOT_BASE);

                lookahead.scale.x               = 0.03;
                lookahead.scale.y               = 0.03;
                lookahead.scale.z               = 0.03;

                lookahead.color                 = potbot_lib::color::get_msg(potbot_lib::color::RED);
                lookahead.color.a               = 0.5;
                
                pub_look_ahead_.publish(lookahead);
            }
            break;
        }
    }

    geometry_msgs::Twist cmd;
    double init_angle, x1,x2,y1,y2;

    if (robot_path_index_ <= robot_path_size - 2)
    {
        x1 = robot_path_.poses[robot_path_index_].pose.position.x;
        x2 = robot_path_.poses[robot_path_index_+1].pose.position.x;
        y1 = robot_path_.poses[robot_path_index_].pose.position.y;
        y2 = robot_path_.poses[robot_path_index_+1].pose.position.y;
    }
    else
    {
        x1 = robot_path_.poses[robot_path_index_-1].pose.position.x;
        x2 = robot_path_.poses[robot_path_index_].pose.position.x;
        y1 = robot_path_.poses[robot_path_index_-1].pose.position.y;
        y2 = robot_path_.poses[robot_path_index_].pose.position.y;
    }
    init_angle = atan2(y2-y1,x2-x1);

    if(!done_init_pose_alignment_ && abs(init_angle - potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation)) > M_PI/6.0)
    {
        geometry_msgs::Quaternion alpha_quat = potbot_lib::utility::get_Quat(0,0,init_angle);
        geometry_msgs::Pose target;
        target.position = robot_.pose.pose.position;
        target.orientation = alpha_quat;
        potbot_lib::utility::print_Pose(target);
        __PoseAlignment(target);
    }
    else if (robot_path_index_ < robot_path_size)
    {   
        done_init_pose_alignment_ = true;
        double yaw = potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation);
        double alpha = atan2(sub_goal.y - robot_.pose.pose.position.y, sub_goal.x - robot_.pose.pose.position.x) - yaw;
        cmd.linear.x = MAX_LINEAR_VELOCITY;
        cmd.angular.z = 2*cmd.linear.x*sin(alpha)/l_d;
        cmd_ = cmd;
        //ROS_INFO("comannd v,omega: %f, %f", cmd_.linear.x, cmd_.angular.z);
    } 
}

void ControllerClass::__PoseAlignment(geometry_msgs::Pose target)
{
    double v=0,omega=0;
    double distance     = potbot_lib::utility::get_Distance(robot_.pose.pose.position, target.position);
    double yaw_now      = potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation);
    double yaw_target   = potbot_lib::utility::get_Yaw(target.orientation);
    // 
    double yaw_err = yaw_target - yaw_now;
    // potbot_lib::utility::print_Pose(target);
    // ROS_INFO("__PoseAlignment target to: %f [m], %f [deg]",distance, yaw_err);
    if (distance > 0.05)
    {
        double yaw_to_target = atan2(target.position.y - robot_.pose.pose.position.y ,target.position.x - robot_.pose.pose.position.x);
        double yaw_to_err = yaw_to_target - yaw_now;
        if (abs(yaw_to_err) > 0.01)
        {
            // yaw_target = potbot_lib::utility::get_Yaw(target.orientation);
            omega = yaw_to_err;
        }
        else
        {
            v = 0.05;
            omega = yaw_to_err;
        }
    }
    else
    {
        if(abs(yaw_err) > 0.01)
        {
            omega = yaw_err;
        }
        
    }
    cmd_.linear.x = v;
    cmd_.angular.z = omega;
    // ROS_INFO("__PoseAlignment (v,omega) = (%f, %f)", cmd_.linear.x, cmd_.angular.z);
}

// void ControllerClass::__PID()
// {
//     double error_distance = get_Distance(robot_.pose.pose.position, goal_.pose.position);
//     double error_angle = get_Yaw(goal_.pose.orientation) - get_Yaw(robot_.pose.pose.orientation);
//     cmd_.linear.x = 0.1*error_distance;
//     cmd_.angular.z = 0.1*error_angle;
// }

void ControllerClass::__publishcmd()
{
    // ROS_INFO("comannd v,omega: %f / %f, %f / %f", cmd_.linear.x, MAX_LINEAR_VELOCITY, cmd_.angular.z, MAX_ANGULAR_VELOCITY);
    if (cmd_.linear.x > MAX_LINEAR_VELOCITY) cmd_.linear.x = MAX_LINEAR_VELOCITY;
    if (cmd_.angular.z > MAX_ANGULAR_VELOCITY) cmd_.angular.z = MAX_ANGULAR_VELOCITY;
    pub_cmd_.publish(cmd_);
}

void ControllerClass::__publish_path_request()
{
    geometry_msgs::PoseStamped goal_init;
    if(goal_init != goal_)
    {
        std_msgs::Empty empty;
        pub_path_request_.publish(empty);
    }
}