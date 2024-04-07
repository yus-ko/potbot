#include<potbot_pathplanner/PathPlanning.h>

PathPlanningClass::PathPlanningClass()
{

	ros::NodeHandle n("~");
    n.getParam("PATH_PLANNING_METHOD",      PATH_PLANNING_METHOD);
    n.getParam("PATH_PLANNING_FILE",        PATH_PLANNING_FILE);
    n.getParam("TARGET/POSITION/X",         TARGET_POSITION_X);
    n.getParam("TARGET/POSITION/Y",         TARGET_POSITION_Y);
    n.getParam("TARGET/POSITION/YAW",       TARGET_POSITION_YAW);
    n.getParam("FRAME_ID/GLOBAL",           FRAME_ID_GLOBAL);
    n.getParam("FRAME_ID/ROBOT_BASE",       FRAME_ID_ROBOT_BASE);
    n.getParam("TOPIC/ODOM",                TOPIC_ODOM);
    n.getParam("TOPIC/GOAL",                TOPIC_GOAL);

    goal_.pose.position.x = TARGET_POSITION_X;
    goal_.pose.position.y = TARGET_POSITION_Y;
    goal_.pose.orientation = potbot_lib::utility::get_Quat(0,0,TARGET_POSITION_YAW);

	if (PATH_PLANNING_METHOD == "csv")
	{
		path_planning_id_ = potbot_lib::CSV_PATH;
	}
	else if (PATH_PLANNING_METHOD == "potential_method")
	{
		path_planning_id_ = potbot_lib::POTENTIAL_METHOD;
	}

	sub_goal_ 		= nhSub.subscribe(TOPIC_GOAL, 1, &PathPlanningClass::goal_callback, this);
	sub_odom_		= nhSub.subscribe(TOPIC_ODOM,1,&PathPlanningClass::__odom_callback,this);
	sub_local_map_	= nhSub.subscribe("Localmap", 1, &PathPlanningClass::local_map_callback, this);
	sub_run_		= nhSub.subscribe("create_path", 1, &PathPlanningClass::__create_path_callback, this);
	sub_seg_		= nhSub.subscribe("segment", 1, &PathPlanningClass::__segment_callback, this);
	sub_state_		= nhSub.subscribe("state",1,&PathPlanningClass::__state_callback,this);
	
	//pub_odom= nhPub.advertise<nav_msgs::Odometry>("/potbot/odom", 1);
	pub_path_			    = nhPub.advertise<nav_msgs::Path>("Path", 1);
	pub_attraction_field_	= nhPub.advertise<sensor_msgs::PointCloud2>("field/attraction", 1);
	pub_repulsion_field_	= nhPub.advertise<sensor_msgs::PointCloud2>("field/repulsion", 1);
	pub_potential_field_	= nhPub.advertise<sensor_msgs::PointCloud2>("field/potential", 1);

	f_ = boost::bind(&PathPlanningClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

	static tf2_ros::TransformListener tfListener(tf_buffer_);

}
PathPlanningClass::~PathPlanningClass(){
}

void PathPlanningClass::goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
    ROS_INFO("subscribe goal: path planner");
    run();
}

void PathPlanningClass::local_map_callback(const nav_msgs::OccupancyGrid& msg)
{
    local_map_ = msg;
    // ROS_INFO("subscribe local map");
    // static ros::Time hit_time = local_map_.header.stamp;
    if(__PathCollision())
    {
        // ROS_INFO("hit");
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
    obstacle_state_ = msg;
}

void PathPlanningClass::__create_path_callback(const std_msgs::Empty& msg)
{
    run();
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
}

#define IS_REPULSION_FIELD_INSIDE 0
#define IS_REPULSION_FIELD_EDGE 1
#define IS_REPULSION_FIELD_EDGE_DUPLICATION 2
#define IS_PLANNED_PATH 3
#define IS_AROUND_GOAL 4

void PathPlanningClass::run()
{
    // geometry_msgs::PoseStamped robot_pose;
    // while (!get_WorldCoordinate(FRAME_ID_ROBOT_BASE, ros::Time(0), robot_pose, tf_buffer_)){}
    // header_ = robot_pose.header;
    // odom_.header = header_;
    // odom_.pose.pose = robot_pose.pose;
    if (path_planning_id_ == potbot_lib::POTENTIAL_METHOD)
    {
        if(__create_PotentialField())
        {
            //__create_Path();
            // __create_Path_used_weight();
        }
    }
    // publishPathPlan();
    
}

std::vector<nav_msgs::Odometry> PathPlanningClass::__get_ObstacleList(int mode)
{
    if (mode == 0)
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
    else if (mode == 1)
    {
        std::vector<nav_msgs::Odometry> obstacle_arr;
        int size = obstacles_.markers.size();
        for (int i = 0; i < size; i++)
        {
            if(obstacles_.markers[i].ns.find("centor") != std::string::npos)
            {
                nav_msgs::Odometry obs;
                obs.pose.pose.position.x = obstacles_.markers[i].pose.position.x;
                obs.pose.pose.position.y = obstacles_.markers[i].pose.position.y;
                obstacle_arr.push_back(obs);
            }
        }
        return obstacle_arr;
    }
    else if (mode == 2)
    {
        std::vector<nav_msgs::Odometry> obstacle_arr;
        for (int i = 0; i < obstacle_state_.data.size(); i++)
        {

            // 変換する座標
            geometry_msgs::PoseStamped world_obstacle, robot_obstacle;
            world_obstacle.header.frame_id = FRAME_ID_GLOBAL;
            world_obstacle.header.stamp = header_.stamp;
            world_obstacle.pose.position.x = obstacle_state_.data[i].xhat.data[0];
            world_obstacle.pose.position.y = obstacle_state_.data[i].xhat.data[1];
            world_obstacle.pose.orientation = potbot_lib::utility::get_Quat(0,0,-obstacle_state_.data[i].xhat.data[2]);

            robot_obstacle.header.frame_id = FRAME_ID_ROBOT_BASE;

            geometry_msgs::TransformStamped transform;
            static tf2_ros::TransformListener tfListener(tf_buffer_);
            try 
            {
                // 世界座標系の障害物位置をロボット座標系に変換
                transform = tf_buffer_.lookupTransform(robot_obstacle.header.frame_id, world_obstacle.header.frame_id, ros::Time());
                tf2::doTransform(world_obstacle, robot_obstacle, transform);
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_ERROR("TF Ereor in %d: %s",i, ex.what());
                continue;
            }

            nav_msgs::Odometry obs;
            obs.header = robot_obstacle.header;
            // obs.pose.pose.position.x = obstacle_state_.data[i].xhat.data[0];
            // obs.pose.pose.position.y = obstacle_state_.data[i].xhat.data[1];
            obs.pose.pose.position.x = robot_obstacle.pose.position.x;
            obs.pose.pose.position.y = robot_obstacle.pose.position.y;
            // obs.pose.pose.orientation = get_Quat(0,0,obstacle_state_.data[i].xhat.data[2]);
            obs.pose.pose.orientation = robot_obstacle.pose.orientation;
            obs.twist.twist.linear.x = obstacle_state_.data[i].xhat.data[3];
            obs.twist.twist.angular.z = obstacle_state_.data[i].xhat.data[4];
            // obs.pose.pose.orientation = get_Quat(0,0,test_theta_);
            // obs.twist.twist.linear.x = test_vx_;
            //ROS_INFO("theta,v = %f, %f",get_Yaw(obs.pose.pose.orientation),obs.twist.twist.linear.x);
            obstacle_arr.push_back(obs);

            double k = 1.0;
            int num = abs(k*obs.twist.twist.linear.x/rho_zero_);
            for (int j = 1; j < num; j++)
            {
                nav_msgs::Odometry obs_add;
                obs_add.header = obs.header;
                
                double l = k*obs.twist.twist.linear.x/num*j;
                obs_add.pose.pose.position.x = l*cos(potbot_lib::utility::get_Yaw(obs.pose.pose.orientation)) + obs.pose.pose.position.x;
                obs_add.pose.pose.position.y = l*sin(potbot_lib::utility::get_Yaw(obs.pose.pose.orientation)) + obs.pose.pose.position.y;
                obs_add.pose.pose.orientation = obs.pose.pose.orientation;
                obs_add.twist = obs.twist;
                ROS_INFO("%d: theta,v = %f, %f",j, potbot_lib::utility::get_Yaw(obs_add.pose.pose.orientation),l);

                obstacle_arr.push_back(obs_add);
            }

        }
        return obstacle_arr;
    }

}

double PathPlanningClass::__get_ShortestDistanceToObstacle(double x, double y, std::vector<geometry_msgs::Vector3> &obstacles)
{
    int size = obstacles.size();
    double shortest_distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < size; i++)
    {
        double ox = obstacles[i].x;
        double oy = obstacles[i].y;
        double distance = sqrt(pow(x-ox,2) + pow(y-oy,2));
        if (distance < shortest_distance)
        {
            shortest_distance = distance;
        }
    }
    return shortest_distance;
}

int PathPlanningClass::__create_PotentialField()
{

    // geometry_msgs::PoseStamped robot_pose = __get_WorldCoordinate(FRAME_ID_ROBOT_BASE,local_map_.header.stamp);
    // // __print_Pose(lidar_pose.pose);
    // double robot_x = robot_pose.pose.position.x;
    // double robot_y = robot_pose.pose.position.y;

    double robot_x = odom_.pose.pose.position.x;
    double robot_y = odom_.pose.pose.position.y;

    double linear_vel = odom_.twist.twist.linear.x;
    double pose = potbot_lib::utility::get_Yaw(odom_.pose.pose.orientation);
    //std::cout<< linear_vel << ", " << robot_pose<<std::endl;
    double robot_vx = linear_vel*cos(pose);
    double robot_vy = linear_vel*sin(pose);
    //std::cout<< robot_vx << ", " << robot_vy<<std::endl;

    int map_size = local_map_.data.size();
    int map_cols = local_map_.info.width;
    int map_rows = local_map_.info.height;
    double map_ori_x = local_map_.info.origin.position.x;
    double map_ori_y = local_map_.info.origin.position.y;
    double map_res = local_map_.info.resolution;

    std::vector<nav_msgs::Odometry> obstacles = __get_ObstacleList(0);  //引数確認

    potential_field_.header = header_;
    potential_field_.header.frame_id = FRAME_ID_ROBOT_BASE;
    potential_field_.cell_width = map_res;
    potential_field_.cell_height = map_res;
    potential_field_.cells.resize(map_size);
    potential_field_info_.resize(map_size);

    std::vector<bool> potential_field_cell_info = {false,false,false,false,false};
    std::fill(potential_field_info_.begin(), potential_field_info_.end(), potential_field_cell_info);

    geometry_msgs::PoseStamped world_goal = goal_;
    world_goal.header.stamp = ros::Time();
    geometry_msgs::PoseStamped robot_goal = potbot_lib::utility::get_tf(tf_buffer_, world_goal, FRAME_ID_ROBOT_BASE);
    
    potbot_lib::PathPlanner::APFPathPlanner apf(
							potential_field_rows_,					//ポテンシャル場の幅(x軸方向) [m]
							potential_field_cols_,					//ポテンシャル場の高さ(y軸方向) [m]
							potential_field_resolution_,			//ポテンシャル場グリッド1辺の長さ [m]
							kp_,				                    //引力場の重み
							eta_,				                    //斥力場の重み
							rho_zero_                               //斥力場を作る距離の閾値 [m]
							);
    apf.set_goal(robot_goal.pose.position.x, robot_goal.pose.position.y);
    apf.set_robot(0,0);
    
    for (auto obs : obstacles)
    {
        apf.set_obstacle(obs.pose.pose.position.x, obs.pose.pose.position.y);
    }

    apf.create_potential_field();

    std::vector<std::vector<double>> path_raw, path_interpolated;
    // double init_yaw = potbot_lib::utility::get_Yaw(g_robot.pose.pose.orientation);
    // if (isnan(init_yaw)) init_yaw = 0;
    double init_yaw = 0;

    apf.create_path(path_raw, init_yaw, max_path_length_, path_search_range_);
    apf.bezier(path_raw, path_interpolated);

    potbot_lib::Potential::Field attraction_field, repulsion_field, potential_field, filtered_field;
    apf.get_attraction_field(attraction_field);
    apf.get_repulsion_field(repulsion_field);
    apf.get_potential_field(potential_field);
    // potential_field.info_filter(filtered_field, {potbot_lib::Potential::GridInfo::IS_PLANNED_PATH, potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_EDGE},"and");
    // potential_field.info_filter(filtered_field, g_potential_field_filter_terms, g_potential_field_filter_mode);

    nav_msgs::Path path_msg_raw, path_msg_interpolated;
    for (auto point : path_raw)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = point[0];
        pose_msg.pose.position.y = point[1];
        pose_msg.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
        path_msg_raw.poses.push_back(pose_msg);
    }
    for (auto point : path_interpolated)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = point[0];
        pose_msg.pose.position.y = point[1];
        pose_msg.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
        path_msg_interpolated.poses.push_back(pose_msg);
    }

    sensor_msgs::PointCloud2 attraction_field_msg, repulsion_field_msg, potential_field_msg, filtered_field_msg;
    attraction_field.to_pcl2(attraction_field_msg);
    repulsion_field.to_pcl2(repulsion_field_msg);
    potential_field.to_pcl2(potential_field_msg);
    // filtered_field.to_pcl2(filtered_field_msg);

    std_msgs::Header header_apf     = robot_goal.header;
    header_apf.stamp                = local_map_.header.stamp;
    attraction_field_msg.header		= header_apf;
    repulsion_field_msg.header		= header_apf;
    potential_field_msg.header		= header_apf;
    // filtered_field_msg.header		= header_apf;
    path_msg_raw.header				= header_apf;
    path_msg_interpolated.header	= header_apf;

    robot_path_                     = path_msg_raw;

    pub_attraction_field_.publish(attraction_field_msg);
    pub_repulsion_field_.publish(repulsion_field_msg);
    pub_potential_field_.publish(potential_field_msg);
    // pub_filtered_field.publish(filtered_field_msg);
    // pub_path_raw.publish(path_msg_raw);
    pub_path_.publish(path_msg_interpolated);

    robot_path_world_coord_.header = robot_path_.header;
    robot_path_world_coord_.header.frame_id = FRAME_ID_GLOBAL;
    robot_path_world_coord_.poses.clear();
    for (const auto& pose : robot_path_.poses)
    {
        geometry_msgs::PoseStamped world_pose;
        geometry_msgs::TransformStamped transform;
        try 
        {
            // ロボット座標系の経路を世界座標系に変換
            transform = tf_buffer_.lookupTransform(robot_path_world_coord_.header.frame_id, robot_path_.header.frame_id, robot_path_.header.stamp);
            tf2::doTransform(pose, world_pose, transform);
            robot_path_world_coord_.poses.push_back(world_pose);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_ERROR("TF Ereor : %s", ex.what());
            continue;
        }
    }

    return potbot_lib::SUCCESS;
}

int PathPlanningClass::__get_PotentialFiledIndex(double x, double y)
{
    int map_size = local_map_.data.size();
    int map_cols = local_map_.info.width;
    int map_rows = local_map_.info.height;
    double map_ori_x = local_map_.info.origin.position.x;
    double map_ori_y = local_map_.info.origin.position.y;
    double map_res = local_map_.info.resolution;
    if (map_res <= 0) return -1;

    double px = x - map_ori_x;
    double py = y - map_ori_y;
    int col = px/map_res;
    int row = py/map_res;
    if (col > map_cols || row > map_rows) return -1;
    int index = col + map_cols*row;
    if (index > map_size || index < 0) return -1;
    return index;
}

void PathPlanningClass::__create_Path_used_weight()
{
    nav_msgs::Path robot_path;
    robot_path.header = header_;
    robot_path.header.frame_id = FRAME_ID_ROBOT_BASE;

    double center_x = 0;
    double center_y = 0;
    int index = -1;
    double J_min_pre;
    double map_res = local_map_.info.resolution;
    while (true)
    {
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.header = header_;
        robot_pose.header.frame_id = FRAME_ID_ROBOT_BASE;
        
        double J_min;
        if (index > -1)
        {
            J_min = J_min_pre;
        }
        else
        {
            J_min = std::numeric_limits<double>::infinity();
        }
        
        double x,y,wu=1,wtheta=0;;
        bool breakflag = false;
        bool local_minimum = true;

        //if (index > -1) ROS_INFO("%f, %f",robot_path.poses[index].pose.position.x, robot_path.poses[index].pose.position.y);

        for (int i = 0; i < 2; i++)
        {
            breakflag = false;
            double J_min_local = std::numeric_limits<double>::infinity();
            for (x = -2*map_res + center_x; x <= 2*map_res + center_x; x+=map_res)
            {
                for (y = -2*map_res + center_y; y <= 2*map_res + center_y; y+=map_res)
                {
                    
                    int pf_idx = __get_PotentialFiledIndex(x,y);
                    if (pf_idx == -1)
                    {
                        breakflag = true;
                        break;
                    }
                    double PotentialValue = potential_field_.cells[pf_idx].z;

                    double posediff;
                    if (index > -1)
                    {
                        double x_pre = robot_path.poses[index].pose.position.x;
                        double y_pre = robot_path.poses[index].pose.position.y;
                        if (x == x_pre && y == y_pre) continue;
                        double theta = potbot_lib::utility::get_Yaw(robot_path.poses[index].pose.orientation);
                        posediff = abs(atan2(y-y_pre,x-x_pre) - theta);
                    }
                    else
                    {
                        posediff = abs(atan2(y,x));
                    }

                    //ROS_INFO("%f",posediff);
                    double J = wu*PotentialValue + wtheta*posediff;

                    if (i == 0)
                    {
                        if (J < J_min) 
                        {
                            local_minimum = false;
                            robot_pose.pose.position.x = x;
                            robot_pose.pose.position.y = y;
                            J_min = J;
                        }
                    }
                    else
                    {
                        if (J < J_min_local) 
                        {
                            local_minimum = false;
                            robot_pose.pose.position.x = x;
                            robot_pose.pose.position.y = y;
                            J_min_local = J;
                        }
                    }
                }
                if (breakflag) break;
            }

            //重み変化
            if (local_minimum)
            {
                //ROS_INFO("wu:%f, wtheta:%f",wu_, w_theta_);
                wu = wu_;
                wtheta = w_theta_;
                //J_min = std::numeric_limits<double>::infinity();
            }
            else
            {
                break;
            }
        }

        if (index > -1)
        {
            if (breakflag || local_minimum) break;
            geometry_msgs::PoseStamped &pre = robot_path.poses.back();
            robot_pose.pose.orientation = potbot_lib::utility::get_Quat(0,0,atan2(robot_pose.pose.position.y-pre.pose.position.y,robot_pose.pose.position.x-pre.pose.position.x));
        }
        else
        {
            robot_pose.pose.orientation = odom_.pose.pose.orientation;
        }
        robot_path.poses.push_back(robot_pose);

        int idxtmp = __get_PotentialFiledIndex(robot_pose.pose.position.x,robot_pose.pose.position.y);
        if (idxtmp != -1) potential_field_info_[idxtmp][IS_PLANNED_PATH] = true;
        index++;
        if (index > max_path_index_ || potential_field_info_[idxtmp][IS_AROUND_GOAL]) break;
        J_min_pre = J_min;
        center_x = robot_pose.pose.position.x;
        center_y = robot_pose.pose.position.y;
    }

    if (index > -1)
    {
        // nav_msgs::Path world_path;
        // world_path.header = header_;
        // world_path.header.frame_id = FRAME_ID_GLOBAL;
        // for (int i = 0; i <= index; i++)
        // {
        //     geometry_msgs::PoseStamped world_pose, target_point;
        //     world_pose.header = world_path.header;

        //     geometry_msgs::TransformStamped transform;
        //     try 
        //     {
        //         // ロボット座標系の経路を世界座標系に変換
        //         transform = tf_buffer_.lookupTransform(world_pose.header.frame_id, robot_path.poses[i].header.frame_id, ros::Time());
        //         tf2::doTransform(robot_path.poses[i], world_pose, transform);
        //     }
        //     catch (tf2::TransformException &ex) 
        //     {
        //         ROS_ERROR("TF Ereor in 2: %s", ex.what());
        //         return;
        //     }

        //     // int break_cnt = 0;
        //     // while(!get_tf(robot_path.poses[i] ,world_pose, tf_buffer_) || break_cnt > 1000){break_cnt++;}
        //     if (potbot_lib::utility::get_Distance(world_pose.pose.position,goal_.pose.position) > 0.06)
        //     {
        //         world_path.poses.push_back(world_pose);
        //     }
        //     else
        //     {
        //         break;
        //     }
        // }
        __bezier(robot_path);
        robot_path_ = robot_path;
    }
    
}

void PathPlanningClass::__create_Path()
{
    nav_msgs::Path robot_path;
    robot_path.header = header_;
    // robot_path.header.stamp = ros::Time(0);
    robot_path.header.frame_id = FRAME_ID_ROBOT_BASE;

    double center_x = 0;
    double center_y = 0;
    int index = -1;
    double J_min_pre;
    double map_res = local_map_.info.resolution;
    while (true)
    {
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.header = header_;
        // robot_pose.header.stamp = ros::Time(0);
        robot_pose.header.frame_id = FRAME_ID_ROBOT_BASE;
        
        double J_min;
        if (index > -1)
        {
            J_min = J_min_pre;
        }
        else
        {
            J_min = std::numeric_limits<double>::infinity();
        }
        
        double x,y,wu=1,wtheta=0;;
        bool breakflag = false;
        bool local_minimum = true;

        for (x = -2*map_res + center_x; x <= 2*map_res + center_x; x+=map_res)
        {
            for (y = -2*map_res + center_y; y <= 2*map_res + center_y; y+=map_res)
            {
                
                int pf_idx = __get_PotentialFiledIndex(x,y);
                if (pf_idx == -1)
                {
                    breakflag = true;
                    break;
                }
                double PotentialValue = potential_field_.cells[pf_idx].z;

                double posediff;
                if (index > -1)
                {
                    double x_pre = robot_path.poses[index].pose.position.x;
                    double y_pre = robot_path.poses[index].pose.position.y;
                    if (x == x_pre && y == y_pre) continue;
                    double theta = potbot_lib::utility::get_Yaw(robot_path.poses[index].pose.orientation);
                    posediff = abs(atan2(y-y_pre,x-x_pre) - theta);
                }
                else
                {
                    posediff = abs(atan2(y,x));
                }

                //ROS_INFO("%f",posediff);
                double J = wu_*PotentialValue + w_theta_*posediff;

                if (J < J_min) 
                {
                    local_minimum = false;
                    robot_pose.pose.position.x = x;
                    robot_pose.pose.position.y = y;
                    J_min = J;
                }
            }
            if (breakflag) break;
        }

        if (local_minimum)
        {
            breakflag = false;
            for (x = -2*map_res + center_x; x <= 2*map_res + center_x; x+=map_res)
            {
                for (y = -2*map_res + center_y; y <= 2*map_res + center_y; y+=map_res)
                {
                    // double x_pre = robot_path.poses[index].pose.position.x;
                    // double y_pre = robot_path.poses[index].pose.position.y;
                    // if (x == x_pre && y == y_pre) continue;

                    int pf_idx = __get_PotentialFiledIndex(x,y);
                    if (pf_idx == -1)
                    {
                        breakflag = true;
                        break;
                    }
                    if (potential_field_info_[pf_idx][IS_REPULSION_FIELD_EDGE] && !potential_field_info_[pf_idx][IS_PLANNED_PATH])
                    {
                        robot_pose.pose.position.x = x;
                        robot_pose.pose.position.y = y;
                        local_minimum = false;
                        break;
                    }
                    
                }
                if (breakflag) break;
            }
        }

        if (index > -1)
        {
            if (breakflag || local_minimum) break;
            geometry_msgs::PoseStamped &pre = robot_path.poses.back();
            robot_pose.pose.orientation = potbot_lib::utility::get_Quat(0,0,atan2(robot_pose.pose.position.y-pre.pose.position.y,robot_pose.pose.position.x-pre.pose.position.x));
        }
        else
        {
            robot_pose.pose.orientation = odom_.pose.pose.orientation;
        }
        robot_path.poses.push_back(robot_pose);

        int idxtmp = __get_PotentialFiledIndex(robot_pose.pose.position.x,robot_pose.pose.position.y);
        if (idxtmp != -1) potential_field_info_[idxtmp][IS_PLANNED_PATH] = true;
        index++;
        if (index > max_path_index_) break;
        J_min_pre = J_min;
        center_x = robot_pose.pose.position.x;
        center_y = robot_pose.pose.position.y;
    }

    if (index > -1)
    {
        __bezier(robot_path);
        robot_path_ = robot_path;
    }
    
}

void PathPlanningClass::__bezier(nav_msgs::Path& points)
{
    // https://www.f.waseda.jp/moriya/PUBLIC_HTML/education/classes/infomath6/applet/fractal/spline/

    nav_msgs::Path points_original = points;
    int n = points_original.poses.size();

    for (int i = 0; i < n; i++)
    {
        points.poses[i].pose.position.x = points.poses[i].pose.position.y = 0.0;
    }

    int bezier_idx = 0;
    double inc = 1.0/double(n*10);
    for (double t = 0.0; t <= 1.0; t += inc)
    {
        points.poses.resize(bezier_idx+1);
        points.poses[bezier_idx].header = points.header;
        for (double i = 0.0; i <= n-1.0; i++)
        {
            points.poses[bezier_idx].pose.position.x += __nCr(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * points_original.poses[int(i)].pose.position.x;
            points.poses[bezier_idx].pose.position.y += __nCr(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * points_original.poses[int(i)].pose.position.y;
        }
        //std::cout<< bezier_idx << " bezier = (" << points[bezier_idx].x << ", " << points[bezier_idx].y << ")" <<std::endl;
        bezier_idx++;
    }
}

double PathPlanningClass::__nCr(double n, double r)
{
    double top = 1.0;
    double bottom = 1.0;

    for(double i = 0.0; i < r; i++)
    {
        top *= n-i;
    }

    for(double i = 0.0; i < r; i++)
    {
        bottom *= i+1.0;
    }
    
    return top/bottom;
}

void spline(std::vector<geometry_msgs::Vector3>& points)
{
    // http://www.yamamo10.jp/yamamoto/lecture/2006/5E/interpolation/interpolation_html/node3.html
    // 高橋大輔.数値計算.岩波書店,1996.pp43-49

    std::vector<geometry_msgs::Vector3> points_original = points;

    int N = points_original.size() - 1;
	Eigen::VectorXd x(N+1);
	Eigen::VectorXd y(N+1);

	for(int j=0; j<N+1; j++)
	{
		x[j] = points_original[j].x;
		y[j] = points_original[j].y;
	}

	Eigen::VectorXd h(N);
	for(int j=0; j<N; j++)
	{
		h[j] = x[j+1] - x[j];
	}

	Eigen::VectorXd v(N-1);
	for(int j=1; j<N; j++)
	{
		v[j-1] = 6*(((y[j+1] - y[j])/h[j]) - ((y[j] - y[j-1])/h[j-1]));
	}

	Eigen::MatrixXd H(N-1, N-1);
	for(int j=0; j<N-1; j++)
	{
		for(int k=0; k<N-1; k++)
		{
			if(j == 0)
			{
				if(k == 0)
				{
					H(j,k) = 2*(h[j] + h[j+1]);
				}
				else if (k == 1)
				{
					H(j,k) = h[j+1];
				}
				else
				{
					H(j,k) = 0.0;
				}
			}
			else if(j == N-2)
			{
				if(k == N-2)
				{
					H(j,k) = 2*(h[j] + h[j+1]);
				}
				else if (k == N-3)
				{
					H(j,k) = h[j];
				}
				else
				{
					H(j,k) = 0.0;
				}
			}
			else
			{
				if(k == j-1)
				{
					H(j,k) = h[j];
				}
				else if (k == j)
				{
					H(j,k) = 2*(h[j] + h[j+1]);
				}
				else if (k == j+1)
				{
					H(j,k) = h[j+1];
				}
				else
				{
					H(j,k) = 0.0;
				}
			}
		}
	}

	Eigen::FullPivLU<Eigen::MatrixXd> LU(H);
	Eigen::VectorXd U = LU.solve(v);
	
	int U_size = U.size();
	Eigen::VectorXd u(U_size+2);
	int idx = 0;
	for (int i = 0; i < U_size+2; i++)
	{
		if(i == 0 || i == U_size+1)
		{
			u(i) = 0.0;
		}
		else
		{
			u(i) = U(idx++);
		}
	}

	Eigen::VectorXd a(N);
	Eigen::VectorXd b(N);
	Eigen::VectorXd c(N);
	Eigen::VectorXd d(N);
	for (int j = 0; j < N; j++)
	{
		a(j) = (u(j+1) - u(j)) / (6*(x(j+1) - x(j)));
		b(j) = u(j) / 2;
		c(j) = ((y(j+1) - y(j)) / (x(j+1) - x(j))) - ((1.0/6.0)*(x(j+1) - x(j))*(2.0*u(j) + u(j+1)));
		d(j) = y(j);
	}

	points.resize(0);
	for (int j = 0; j < N; j++)
	{
        double increase = (x(j+1) - x(j)) / 10;
		for (double t = x(j); t < x(j+1); t+=increase)
		{
			geometry_msgs::Vector3 ans;
			ans.x = t;
			// if (j == 0 && t == x(j))
			// {
			// 	ans.y = ;
			// }
			// else if ()
			// {

			// }
			// else
			// {

			// }
			double x_xj = t - x(j);
			ans.y = a(j)*pow(x_xj,3) + b(j)*pow(x_xj,2) + c(j)*x_xj + d(j);
			points.push_back(ans);
		}
	}

}

bool PathPlanningClass::__PathCollision()
{
    if (robot_path_world_coord_.poses.empty()) return false;

    std::vector<nav_msgs::Odometry> obstacles = __get_ObstacleList(0);
    for (int i = 0; i < obstacles.size(); i++)
    {
        geometry_msgs::Pose world_obslacle_pose;
        geometry_msgs::TransformStamped transform;
        static tf2_ros::TransformListener tfListener(tf_buffer_);
        try 
        {
            // ロボット座標系の障害物を世界座標系に変換
            transform = tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, obstacles[i].header.frame_id, obstacles[i].header.stamp);
            tf2::doTransform(obstacles[i].pose.pose, world_obslacle_pose, transform);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_ERROR("TF Ereor : %s", ex.what());
            continue;
        }

        for (int j = 0; j < robot_path_world_coord_.poses.size(); j++)
        {

            if (potbot_lib::utility::get_Distance(world_obslacle_pose.position, robot_path_world_coord_.poses[j].pose.position) < hit_distance_to_replanning_)
            // if (potbot_lib::utility::get_Distance(obstacles[i].pose.pose.position, robot_path_.poses[j].pose.position) < 0.2)
            {
                return true;
            }
            
        }
    }
    return false;
}

void PathPlanningClass::publishPathPlan()
{
    pub_path_.publish(robot_path_);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_pp");

    PathPlanningClass rcc;
	ros::spin();

	return 0;
}