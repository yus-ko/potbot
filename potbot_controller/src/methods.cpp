#include<potbot_controller/Controller.h>

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
    // ROS_INFO("path_size: %d", robot_path_.poses.size());
    //potbot_lib::utility::print_Pose(goal_.pose);
    if (robot_path_.poses.size() > 0) controller();
    else __publish_path_request();
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
        if (COLLISION_DETECTION && abs(robot_.twist.twist.angular.z) < 0.1 &&  __PathCollision())
        {
            ROS_INFO("PathCollision");
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
    //ROS_INFO("line following processing: %3.1f %% index:%d/%d Done", robot_path_index_, robot_path_size, procces*100);
    if (procces > 0.8 && potbot_lib::utility::get_Distance(robot_path_.poses[robot_path_size-1].pose.position, goal_.pose.position) > 0.1)
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
            if (l_d > 1)
            {
                __publish_path_request();
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
        geometry_msgs::Quaternion alpha_quat;
        potbot_lib::utility::getQuat(0,0,init_angle,alpha_quat);
        geometry_msgs::Pose target;
        target.position = robot_.pose.pose.position;
        target.orientation = alpha_quat;
        //print_Pose(target);
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
    double v=0,omega=0,yaw_target,yaw_err;
    double yaw_now = potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation);
    if (potbot_lib::utility::get_Distance(robot_.pose.pose.position, target.position) < 0.05)
    {
        if (abs(yaw_err) > 0.01)
        {
            yaw_target = potbot_lib::utility::get_Yaw(target.orientation);
            yaw_err = yaw_target - yaw_now;
            omega = yaw_err;
        }
    }
    else
    {
        yaw_target = atan2(target.position.y - robot_.pose.pose.position.y ,target.position.x - robot_.pose.pose.position.x);
        yaw_err = yaw_target - yaw_now;
        v = 0.05;
        omega = yaw_err;
    }
    cmd_.linear.x = v;
    cmd_.angular.z = omega;
}

bool ControllerClass::__PathCollision()
{
    static int mode = 0;
    if (mode == 0)
    {
        int path_size = robot_path_.poses.size();
        
        int map_size = local_map_.data.size();

        std::vector<geometry_msgs::Point> obs;
        double rx = robot_.pose.pose.position.x;
        double ry = robot_.pose.pose.position.y;
        double yaw = potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation);

        int scan_size = scan_.ranges.size();
        for (int i = 0; i < scan_size; i++)
        {
            if (!std::isinf(scan_.ranges[i]) && !std::isnan(scan_.ranges[i]))
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
                if (potbot_lib::utility::get_Distance(obs[i],robot_path_.poses[p].pose.position) < 0.15)
                {
                    // geometry_msgs::Pose pose;
                    // pose.position = obs[i];
                    // std::cout<<"obs";
                    // print_Pose(pose);
                    // std::cout<<"path";
                    // print_Pose(robot_path_.poses[p].pose);
                    return true;
                }
            }
        }

        return false;
    }
    else if (mode == 1)
    {
        std::vector<geometry_msgs::Vector3> obstacle_arr;
        for (int i = 0; i < obstacle_segment_.markers.size(); i++)
        {
            if (obstacle_segment_.markers[i].ns == "segments_display")
            {

                geometry_msgs::Pose world_obslacle_pose;
                geometry_msgs::TransformStamped transform;
                static tf2_ros::TransformListener tfListener(tf_buffer_);
                try 
                {
                    // ロボット座標系の障害物を世界座標系に変換
                    transform = tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, obstacle_segment_.markers[i].header.frame_id, ros::Time());
                    tf2::doTransform(obstacle_segment_.markers[i].pose, world_obslacle_pose, transform);
                }
                catch (tf2::TransformException &ex) 
                {
                    ROS_ERROR("TF Ereor : %s", ex.what());
                    continue;
                }

                if (obstacle_segment_.markers[i].type == visualization_msgs::Marker::SPHERE)
                {
                    double a = world_obslacle_pose.position.x;
                    double b = world_obslacle_pose.position.y;
                    double r = obstacle_segment_.markers[i].scale.x/2.0;
                    for (int p = robot_path_index_; p < robot_path_.poses.size(); p++)
                    {
                        double x = robot_path_.poses[p].pose.position.x;
                        double y = robot_path_.poses[p].pose.position.y;
                        if (x <= sqrt(pow(r,2)-pow(y-b,2))+a && y <= sqrt(pow(r,2)-pow(x-a,2))+b) return true;
                    }
                }
                else if(obstacle_segment_.markers[i].type == visualization_msgs::Marker::CUBE)
                {
                    double xmin = world_obslacle_pose.position.x - obstacle_segment_.markers[i].scale.x/2.0;
                    double xmax = world_obslacle_pose.position.x + obstacle_segment_.markers[i].scale.x/2.0;
                    double ymin = world_obslacle_pose.position.y - obstacle_segment_.markers[i].scale.y/2.0;
                    double ymax = world_obslacle_pose.position.y + obstacle_segment_.markers[i].scale.y/2.0;
                    for (int p = robot_path_index_; p < robot_path_.poses.size(); p++)
                    {
                        double x = robot_path_.poses[p].pose.position.x;
                        double y = robot_path_.poses[p].pose.position.y;
                        if (x >= xmin && x <= xmax && y >= ymin && y <= ymax) return true;
                    }
                }
                
            }
        }
        return false;
    }
    
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
    //ROS_INFO("comannd v,omega: %f, %f", cmd_.linear.x, cmd_.angular.z);
    pub_cmd_.publish(cmd_);
}

void ControllerClass::__publish_path_request()
{
    std_msgs::Empty empty;
    pub_path_request_.publish(empty);
}