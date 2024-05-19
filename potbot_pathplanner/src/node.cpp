#include<potbot_pathplanner/PathPlanning.h>

PathPlanningClass::PathPlanningClass()
{

	ros::NodeHandle n("~");
    n.getParam("frame_id_global",           frame_id_global_);
    n.getParam("frame_id_robot_base",       frame_id_robot_base_);
    n.getParam("topic_goal",                topic_goal_);

	sub_goal_ 		= nhSub.subscribe(topic_goal_,      1, &PathPlanningClass::__goal_callback, this);
	sub_local_map_	= nhSub.subscribe("Localmap",       1, &PathPlanningClass::__local_map_callback, this);
	sub_run_		= nhSub.subscribe("create_path",    1, &PathPlanningClass::__create_path_callback, this);
	
	//pub_odom= nhPub.advertise<nav_msgs::Odometry>("/potbot/odom", 1);
	pub_path_			    = nhPub.advertise<nav_msgs::Path>("Path", 1);
    pub_path_raw_		    = nhPub.advertise<nav_msgs::Path>("debug/Path_raw", 1);
	pub_attraction_field_	= nhPub.advertise<sensor_msgs::PointCloud2>("field/attraction", 1);
	pub_repulsion_field_	= nhPub.advertise<sensor_msgs::PointCloud2>("field/repulsion", 1);
	pub_potential_field_	= nhPub.advertise<sensor_msgs::PointCloud2>("field/potential", 1);

	f_ = boost::bind(&PathPlanningClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

	static tf2_ros::TransformListener tfListener(tf_buffer_);

}
PathPlanningClass::~PathPlanningClass(){
}

void PathPlanningClass::__goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal: path planner");
    if (!sync_create_path_) __create_Path();
}

void PathPlanningClass::__local_map_callback(const nav_msgs::OccupancyGrid& msg)
{
    local_map_ = msg;
    // ROS_INFO("subscribe local map");
    // static ros::Time hit_time = local_map_.header.stamp;

    
    if(sync_create_path_)
    {
        __create_Path();
    }
    else if (sync_create_apf_)
    {
        __create_PotentialField();
    }
    else
    {
        if(__PathCollision())
        {
            hit_count_+=1;
            ROS_INFO("path collide: %d / %d", hit_count_, collision_count_to_replanning_);
            if(hit_count_ >= collision_count_to_replanning_)
            {
                hit_count_ = 0;
                __create_Path();
            }
        }
        else
        {
            hit_count_ = 0;
        }
    }
}

void PathPlanningClass::__create_path_callback(const std_msgs::Empty& msg)
{
    __create_Path();
}

void PathPlanningClass::__param_callback(const potbot_msgs::PathPlanningConfig& param, uint32_t level)
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

    collision_count_to_replanning_  = param.collision_count_to_replanning;
    hit_distance_to_replanning_     = param.hit_distance_to_replanning;

    sync_create_path_               = param.sync_create_path;
    sync_create_apf_                = param.sync_create_apf;
}

std::vector<nav_msgs::Odometry> PathPlanningClass::__get_ObstacleList()
{
    int map_size = local_map_.data.size();
    int map_cols = local_map_.info.width;
    int map_rows = local_map_.info.height;
    double map_ori_x = local_map_.info.origin.position.x;
    double map_ori_y = local_map_.info.origin.position.y;
    double map_res = local_map_.info.resolution;

    std_msgs::Header map_header = local_map_.header;

    std::vector<nav_msgs::Odometry> obstacle_arr(map_size);
    int obs_idx = 0;
    for (int i = 0; i < map_size; i++)
    {
        if (local_map_.data[i])
        {
            obstacle_arr[obs_idx].header = map_header;
            obstacle_arr[obs_idx].pose.pose.position.x = int(i%map_cols)*map_res + map_ori_x;
            obstacle_arr[obs_idx].pose.pose.position.y = int(i/map_cols)*map_res + map_ori_y;
            obs_idx++;
        }
    }
    obstacle_arr.resize(obs_idx);
    return obstacle_arr;

}

int PathPlanningClass::__create_PotentialField()
{

    geometry_msgs::PoseStamped robot_pose = potbot_lib::utility::get_Pose_from_tf(tf_buffer_, local_map_.header.frame_id, frame_id_robot_base_);

    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;

    int map_size = local_map_.data.size();
    int map_cols = local_map_.info.width;
    int map_rows = local_map_.info.height;
    double map_ori_x = local_map_.info.origin.position.x;
    double map_ori_y = local_map_.info.origin.position.y;
    double map_res = local_map_.info.resolution;

    std::vector<nav_msgs::Odometry> obstacles = __get_ObstacleList();

    potential_field_.header = local_map_.header;
    potential_field_.header.frame_id = frame_id_robot_base_;
    potential_field_.cell_width = map_res;
    potential_field_.cell_height = map_res;
    potential_field_.cells.resize(map_size);
    potential_field_info_.resize(map_size);

    std::vector<bool> potential_field_cell_info = {false,false,false,false,false};
    std::fill(potential_field_info_.begin(), potential_field_info_.end(), potential_field_cell_info);

    geometry_msgs::PoseStamped world_goal;
    if (goal_.header.frame_id == "")
    {
        world_goal = robot_pose;
    }
    else
    {
        world_goal = goal_;
    }
    
    apf_ = new potbot_lib::PathPlanner::APFPathPlanner(
							potential_field_rows_,					//ポテンシャル場の幅(x軸方向) [m]
							potential_field_cols_,					//ポテンシャル場の高さ(y軸方向) [m]
							potential_field_resolution_,			//ポテンシャル場グリッド1辺の長さ [m]
							kp_,				                    //引力場の重み
							eta_,				                    //斥力場の重み
							rho_zero_,                              //斥力場を作る距離の閾値 [m]
                            robot_x,                                //ポテンシャル場の中心位置(x軸方向) [m]
                            robot_y                                 //ポテンシャル場の中心位置(y軸方向) [m]
							);
    apf_->set_goal(world_goal.pose.position.x, world_goal.pose.position.y);
    apf_->set_robot(robot_x,robot_y);
    
    for (const auto obs : obstacles)
    {
        apf_->set_obstacle(obs.pose.pose.position.x, obs.pose.pose.position.y);
    }

    apf_->create_potential_field();

    potbot_lib::Potential::Field attraction_field, repulsion_field, potential_field, filtered_field;
    apf_->get_attraction_field(attraction_field);
    apf_->get_repulsion_field(repulsion_field);
    apf_->get_potential_field(potential_field);
    // potential_field.info_filter(filtered_field, {potbot_lib::Potential::GridInfo::IS_PLANNED_PATH, potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_EDGE},"and");
    // potential_field.info_filter(filtered_field, g_potential_field_filter_terms, g_potential_field_filter_mode);

    sensor_msgs::PointCloud2 attraction_field_msg, repulsion_field_msg, potential_field_msg, filtered_field_msg;
    attraction_field.to_pcl2(attraction_field_msg);
    repulsion_field.to_pcl2(repulsion_field_msg);
    potential_field.to_pcl2(potential_field_msg);
    // filtered_field.to_pcl2(filtered_field_msg);

    std_msgs::Header header_apf     = local_map_.header;
    attraction_field_msg.header		= header_apf;
    repulsion_field_msg.header		= header_apf;
    potential_field_msg.header		= header_apf;
    // filtered_field_msg.header		= header_apf;

    pub_attraction_field_.publish(attraction_field_msg);
    pub_repulsion_field_.publish(repulsion_field_msg);
    pub_potential_field_.publish(potential_field_msg);
    // pub_filtered_field.publish(filtered_field_msg);

    return potbot_lib::SUCCESS;
}

void PathPlanningClass::__create_Path()
{
    ROS_INFO("status: create apf");
    __create_PotentialField();

    std::vector<std::vector<double>> path_raw, path_interpolated;
    // double init_yaw = potbot_lib::utility::get_Yaw(g_robot.pose.pose.orientation);
    // if (isnan(init_yaw)) init_yaw = 0;
    double init_yaw = 0;

    ROS_INFO("status: create path");
    apf_->create_path(path_raw, init_yaw, max_path_length_, path_search_range_);

    nav_msgs::Path path_msg_raw;
    path_msg_raw.header				= local_map_.header;
    for (auto point : path_raw)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = point[0];
        pose_msg.pose.position.y = point[1];
        pose_msg.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
        path_msg_raw.poses.push_back(pose_msg);
    }
    pub_path_raw_.publish(path_msg_raw);

    ROS_INFO("status: interpolate");
    apf_->bezier(path_raw, path_interpolated);

    nav_msgs::Path path_msg_interpolated;
    path_msg_interpolated.header	= local_map_.header;
    
    for (auto point : path_interpolated)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = point[0];
        pose_msg.pose.position.y = point[1];
        pose_msg.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
        path_msg_interpolated.poses.push_back(pose_msg);
    }

    robot_path_                     = path_msg_raw;

    pub_path_.publish(path_msg_interpolated);
}

bool PathPlanningClass::__PathCollision()
{
    if (robot_path_.poses.empty()) return false;

    std::vector<nav_msgs::Odometry> obstacles = __get_ObstacleList();
    for (const auto o : obstacles)
    {
        for (const auto p : robot_path_.poses)
        {
            if (potbot_lib::utility::get_Distance(p.pose.position, o.pose.pose.position) < hit_distance_to_replanning_)
            {
                return true;
            }
        }
    }
    return false;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_pp");

    PathPlanningClass rcc;
	ros::spin();

	return 0;
}