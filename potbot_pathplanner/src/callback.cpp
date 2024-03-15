#include<potbot_pathplanner/PathPlanning.h>

void PathPlanningClass::__scan_callback(const sensor_msgs::LaserScan& msg)
{
    scan = msg;
    scan_first = true;
}

void PathPlanningClass::goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal: path planner");
    //std::cout<< goal_ <<std::endl;
    potbot_lib::utility::print_Pose(goal_.pose);
    run();
}

void PathPlanningClass::local_map_callback(const nav_msgs::OccupancyGrid& msg)
{
    local_map_ = msg;
    // ROS_INFO("subscribe local map");
    // static ros::Time hit_time = local_map_.header.stamp;
    if(__PathCollision())
    {
        ROS_INFO("hit");
        hit_count_+=1;
        if(hit_count_ > collision_count_to_replanning_)
        {
            hit_count_ = 0;
            run();
        }
        
        
    }
    else
    {
        hit_count_ = 0;
    }
}

void PathPlanningClass::__odom_callback(const nav_msgs::Odometry& msg)
{
    odom_ = msg;
    header_ = odom_.header;
    //header_.stamp = ros::Time();
}

void PathPlanningClass::__segment_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles_ = msg;
}

void PathPlanningClass::__state_callback(const potbot_msgs::StateArray& msg)
{
    //ROS_INFO("__state_callback");
    obstacle_state_ = msg;
    // for (int i = 0; i < obstacle_state_.data.size(); i++)
    // {
    //     int id          = obstacle_state_.data[i].id;
    //     double x        = obstacle_state_.data[i].xhat.data[0];
    //     double y        = obstacle_state_.data[i].xhat.data[1];
    //     double theta    = obstacle_state_.data[i].xhat.data[2];
    //     double v        = obstacle_state_.data[i].xhat.data[3];
    //     double omega    = obstacle_state_.data[i].xhat.data[4];
    //     ROS_INFO("id: %d (x,y,theta,v,omega) = (%f, %f, %f, %f, %f)",id,x,y,theta,v,omega);
    //     // std::cout<< id << ":(" << v << ", " << omega << ")" <<std::endl;
    // }
    // std::cout<<std::endl;
}

void PathPlanningClass::__create_path_callback(const std_msgs::Empty& msg)
{
    
    double distance = potbot_lib::utility::get_Distance(odom_.pose.pose.position, goal_.pose.position);
    double angle = potbot_lib::utility::get_Yaw(goal_.pose.orientation) - potbot_lib::utility::get_Yaw(odom_.pose.pose.orientation);
    if (distance > 0.05 || abs(angle) > 0.03) run();
}

void PathPlanningClass::__param_callback(const potbot_pathplanner::PathPlanningConfig& param, uint32_t level)
{
    // ROS_INFO("%d",level);

    potential_field_rows_           = param.potential_field_rows;
    potential_field_cols_           = param.potential_field_cols;
    potential_field_resolution_     = param.potential_field_resolution;

    rho_zero_                       = param.distance_threshold_repulsion_field;
    eta_                            = param.weight_repulsion_field;
    kp_                             = param.weight_attraction_field;

    path_search_range_              = param.path_search_range;

    max_path_length_                = param.max_path_length;
    wu_                             = param.weight_potential_field;
    w_theta_                        = param.weight_angle;

    sync_createpath_                = param.sync_createpath_and_controlcycle;

    collision_count_to_replanning_  = param.collision_count_to_replanning;
    hit_distance_to_replanning_     = param.hit_distance_to_replanning;

    test_vx_ = param.test_vx;
    test_vy_ = param.test_vy;
    test_theta_ = param.test_theta/180*M_PI;
    a_ = param.a;
    b_ = param.b;
    c_ = param.c;
}