#include<potbot/PathPlanning.h>

void PathPlanningClass::encoder_callback(const geometry_msgs::Twist& msg)
{
    encoder_value = msg;
    encoder_first = true;
}

void PathPlanningClass::pwcs_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    pwcs_msg = msg;
    odom.pose.pose.position = msg.pose.pose.position;

    Eigen::Quaternionf quat(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);

    Eigen::Matrix3f m2=quat.toRotationMatrix();
	Eigen::Vector3f ea = m2.eulerAngles(1, 2, 0); 
	// std::cout<<ea(0)*180.0/M_PI<<",";//roll
	// std::cout<<ea(1)*180.0/M_PI<<",";//pitch
	// std::cout<<ea(2)*180.0/M_PI<<std::endl;//yaw

    odom.pose.pose.orientation.z = ea(2);

    // std::cout<< "pwcs_callback = "<<std::endl;
    // std::cout<< odom.pose.pose <<std::endl;
    encoder_first = true;
}

void PathPlanningClass::encoder_callback_sim(const nav_msgs::Odometry& msg)
{
    return;
    header_ = msg.header;
    odom_msg = msg;
    odom = msg;
    //odom_ = msg;
    //ROS_INFO("%s",odom_.header.frame_id.c_str());
    encoder_value = msg.twist.twist;
    encoder_first = true;
}

void PathPlanningClass::scan_callback(const sensor_msgs::LaserScan& msg)
{
    scan = msg;
    scan_first = true;
}

void PathPlanningClass::coefficient_callback(const std_msgs::Float32& msg)
{
    coe_0 = msg.data;
}

void PathPlanningClass::cluster_callback(const potbot::ClassificationVelocityData& msg)
{
    pcl_cluster = msg;
}

void PathPlanningClass::goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal");
    //std::cout<< goal_ <<std::endl;
    print_Pose(goal_.pose);
    pub_goal_.publish(goal_);
    // run();
    // publishPathPlan();
}

void PathPlanningClass::local_map_callback(const nav_msgs::OccupancyGrid& msg)
{
    local_map_ = msg;
    // ROS_INFO("subscribe local map");
}

void PathPlanningClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
}

// void PathPlanningClass::__obstacle_callback(const visualization_msgs::MarkerArray& msg)
// {
//     obstacles_ = msg;
// }

void PathPlanningClass::__obstacle_callback(const potbot::StateArray& msg)
{
    potbot::StateArray obstacle_state = msg;
    for (int i = 0; i < obstacle_state.data.size(); i++)
    {
        int id          = obstacle_state.data[i].id;
        double x        = obstacle_state.data[i].xhat.data[0];
        double y        = obstacle_state.data[i].xhat.data[1];
        double theta    = obstacle_state.data[i].xhat.data[2];
        double v        = obstacle_state.data[i].xhat.data[3];
        double omega    = obstacle_state.data[i].xhat.data[4];
        std::cout<< id << ":(" << v << ", " << omega << ")" <<std::endl;
    }
    std::cout<<std::endl;
    // obstacles_ = msg;
}

void PathPlanningClass::__create_path_callback(const std_msgs::Empty& msg)
{
    run();
}

void PathPlanningClass::__param_callback(const potbot::PathPlanningConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);
    rho_zero_ = param.threshold_create_potential_field;
    eta_ = param.weight_obstacle;
    kp_ = param.weight_destination;

    max_path_index_ = param.max_path_length;
    wu_ = param.weight_potential_field;
    w_theta_ = param.weight_angle;
}