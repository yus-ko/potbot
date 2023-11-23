#include<potbot_pathplanner/PathPlanning.h>

#define IS_REPULSION_FIELD_INSIDE 0
#define IS_REPULSION_FIELD_EDGE 1
#define IS_REPULSION_FIELD_EDGE_DUPLICATION 2
#define IS_PLANNED_PATH 3
#define IS_AROUND_GOAL 4

void PathPlanningClass::mainloop()
{
    ros::Rate loop_rate(10);
	while (ros::ok())
	{
        //if (sync_createpath_ || __PathCollision()) run();
        if (sync_createpath_) run();
        
        loop_rate.sleep();
		ros::spinOnce();
	}
}

void PathPlanningClass::run()
{
    // geometry_msgs::PoseStamped robot_pose;
    // while (!get_WorldCoordinate(FRAME_ID_ROBOT_BASE, ros::Time(0), robot_pose, tf_buffer_)){}
    // header_ = robot_pose.header;
    // odom_.header = header_;
    // odom_.pose.pose = robot_pose.pose;
    if (path_planning_id == potbot_lib::POTENTIAL_METHOD)
    {
        if(__create_PotentialField())
        {
            //__create_Path();
            __create_Path_used_weight();
        }
    }
    else if (path_planning_id == potbot_lib::CSV_PATH)
    {
        __create_PathFromCSV();
    }
    publishPathPlan();
    
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

        std::vector<nav_msgs::Odometry> obstacle_arr(map_size);
        int obs_idx = 0;
        for (int i = 0; i < map_size; i++)
        {
            if (local_map_.data[i])
            {
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
            if(obstacles_.markers[i].ns == "segments_display")
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

            double k = a_;
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

    std::vector<nav_msgs::Odometry> obstacles = __get_ObstacleList(1);  //引数確認

    potential_field_.header = header_;
    potential_field_.header.frame_id = FRAME_ID_ROBOT_BASE;
    potential_field_.cell_width = map_res;
    potential_field_.cell_height = map_res;
    potential_field_.cells.resize(map_size);
    potential_field_info_.resize(map_size);

    std::vector<bool> potential_field_cell_info = {false,false,false,false,false};
    std::fill(potential_field_info_.begin(), potential_field_info_.end(), potential_field_cell_info);

    // 変換する座標
    geometry_msgs::PoseStamped world_goal, robot_goal;
    world_goal.header.frame_id = FRAME_ID_GLOBAL;
    world_goal.header.stamp = header_.stamp;
    world_goal.pose = goal_.pose;

    robot_goal.header.frame_id = FRAME_ID_ROBOT_BASE;

    geometry_msgs::TransformStamped transform;
    geometry_msgs::PointStamped target_point;
    static tf2_ros::TransformListener tfListener(tf_buffer_);
    try 
    {
        // 世界座標系のゴールをロボット座標系に変換
        transform = tf_buffer_.lookupTransform(robot_goal.header.frame_id, world_goal.header.frame_id, ros::Time());
        tf2::doTransform(world_goal, robot_goal, transform);
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_ERROR("TF Ereor in pathplan goal: %s", ex.what());
        return potbot_lib::FAIL;
    }

    // int break_cnt = 0;
    // while(!get_tf(world_goal ,robot_goal, tf_buffer_) || break_cnt > 1000){break_cnt++;}
    
    for (int i = 0; i < map_size; i++)
    {
        double x = int(i%map_cols)*map_res + map_ori_x;
        double y = int(i/map_cols)*map_res + map_ori_y;

        // double rho = __get_ShortestDistanceToObstacle(x,y,obstacles) + 0.00000000000001;

        double Uo = 0;
        for (int j = 0; j < obstacles.size(); j++)
        {
            double obs_x        = obstacles[j].pose.pose.position.x;
            double obs_y        = obstacles[j].pose.pose.position.y;
            // double obs_theta    = get_Yaw(obstacles[j].pose.pose.orientation);
            // double obs_v        = obstacles[j].twist.twist.linear.x;
            // double obs_omega    = obstacles[j].twist.twist.angular.z;
            // double obs_vx       = obs_v*cos(obs_theta);
            // double obs_vy       = obs_v*sin(obs_theta);
            // double c = abs(test_vx_) + rho_zero_;
            // double d = abs(test_vy_) + rho_zero_;
            // double e = 1;

            // double rot = test_theta_;

            // double x_rot = x*cos(rot)-y*sin(rot);
            // double y_rot = x*sin(rot)+y*cos(rot);

            // double obs_x_rot = obs_x*cos(rot)-obs_y*sin(rot);
            // double obs_y_rot = obs_x*sin(rot)+obs_y*cos(rot);

            // double edge_x_min = -sqrt(pow(c,2)-pow(c,2)*pow(y_rot-obs_y_rot,2)/pow(d,2))+obs_x_rot;
            // double edge_x_max = sqrt(pow(c,2)-pow(c,2)*pow(y_rot-obs_y_rot,2)/pow(d,2))+obs_x_rot;
            // double edge_y_min = -sqrt(pow(d,2)-pow(d,2)*pow(x_rot-obs_x_rot,2)/pow(c,2))+obs_y_rot;
            // double edge_y_max = sqrt(pow(d,2)-pow(d,2)*pow(x_rot-obs_x_rot,2)/pow(c,2))+obs_y_rot;

            double rho = sqrt(pow(x-obs_x,2) + pow(y-obs_y,2)) + 1e-9;
            
            // double edge_x_min_rot = cos(rot)*edge_x_min-sin(rot)*edge_y_min;
            // double edge_y_min_rot = sin(rot)*edge_x_min+cos(rot)*edge_y_min;
            // double edge_x_max_rot = cos(rot)*edge_x_max-sin(rot)*edge_y_max;
            // double edge_y_max_rot = sin(rot)*edge_x_max+cos(rot)*edge_y_max;
            // if (x >= edge_x_min_rot && x <= edge_x_max_rot && y >= edge_y_min_rot && y <= edge_y_max_rot)
            // if (x_rot >= edge_x_min && x_rot <= edge_x_max && y_rot >= edge_y_min && y_rot <= edge_y_max)
            // if (pow((x_rot-obs_x_rot)/a_,2)+pow(b_*(y_rot-obs_y_rot)/(1+((x_rot-obs_x_rot)+a_)/c_),2) <= 1)
            // if (x >= obs_x-a_ && x <= obs_x+a_ && y >= obs_y-b_ && y <= obs_y+b_)
            if (rho <= rho_zero_)
            {
                potential_field_info_[i][IS_REPULSION_FIELD_INSIDE] = true;
                Uo += 0.5 * eta_ * pow(1/rho - 1/rho_zero_, 2.0);
                // Uo = 1;

                // if ((x >= edge_x_min - map_res && x <= edge_x_min + map_res) || 
                //     (x >= edge_x_max - map_res && x <= edge_x_max + map_res) || 
                //     (y >= edge_y_min - map_res && y <= edge_y_min + map_res) ||
                //     (y >= edge_y_max - map_res && y <= edge_y_max + map_res))
                // {
                //     potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
                // }
                //if (potential_field_info_[i][IS_REPULSION_FIELD_EDGE] && !potential_field_info_[i][IS_REPULSION_FIELD_EDGE_DUPLICATION]) Uo = 2;
            }
            else
            {
                Uo += 0;
            }
        }

        double distance_to_goal = sqrt(pow(x - robot_goal.pose.position.x,2)+pow(y - robot_goal.pose.position.y,2));
        if (distance_to_goal < 0.3) potential_field_info_[i][IS_AROUND_GOAL] = true;
        double Ud = 0.5 * kp_ * pow(distance_to_goal, 2);

        double U = Ud + Uo;
        // if (isnan(U) || isinf(U))
        // {
        //     ROS_INFO("%f, %f, %f, %f",x,y,Ud,Uo);
        // }

        potential_field_.cells[i].x = x;
        potential_field_.cells[i].y = y;
        potential_field_.cells[i].z = U;

    }

    for (int i = map_cols+1; i < map_size-map_cols-1; i++)
    {
        if (potential_field_info_[i][IS_REPULSION_FIELD_INSIDE])
        {
            // double xx = int(i%map_cols)*map_res + map_ori_x;
            // double yy = int(i/map_cols)*map_res + map_ori_y;
            if(!potential_field_info_[i-1][IS_REPULSION_FIELD_INSIDE]) potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
            if(!potential_field_info_[i+1][IS_REPULSION_FIELD_INSIDE]) potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
            if(!potential_field_info_[i-map_cols][IS_REPULSION_FIELD_INSIDE]) potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
            if(!potential_field_info_[i-map_cols-1][IS_REPULSION_FIELD_INSIDE]) potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
            if(!potential_field_info_[i-map_cols+1][IS_REPULSION_FIELD_INSIDE]) potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
            if(!potential_field_info_[i+map_cols][IS_REPULSION_FIELD_INSIDE]) potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
            if(!potential_field_info_[i+map_cols-1][IS_REPULSION_FIELD_INSIDE]) potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
            if(!potential_field_info_[i+map_cols+1][IS_REPULSION_FIELD_INSIDE]) potential_field_info_[i][IS_REPULSION_FIELD_EDGE] = true;
            // bool inside = false;
            // for (int j = 0; j < obstacles.size(); j++)
            // {
            //     double obs_x        = obstacles[j].pose.pose.position.x;
            //     double obs_y        = obstacles[j].pose.pose.position.y;
            //     double c = rho_zero_;
            //     double d = 1;

            //     double edge_x_min = -sqrt(pow(c,2)-pow(c,2)*pow(y-obs_y,2)/pow(c,2))+obs_x;
            //     double edge_x_max = sqrt(pow(c,2)-pow(c,2)*pow(y-obs_y,2)/pow(d,2))+obs_x;
            //     double edge_y_min = -sqrt(pow(d,2)-pow(d,2)*pow(x-obs_x,2)/pow(c,2))+obs_y;
            //     double edge_y_max = sqrt(pow(d,2)-pow(d,2)*pow(x-obs_x,2)/pow(c,2))+obs_y;
            //     if (x >= edge_x_min && x <= edge_x_max && y >= edge_y_min && y <= edge_y_max)
            //     {
            //         if (inside) 
            //         {
            //             potential_field_info_[i][IS_REPULSION_FIELD_EDGE_DUPLICATION] = true;
            //             break;
            //         }
            //         inside = true;
            //     }
            // }
        }
    }
    // for (int i = 1; i < map_size; i++) if (potential_field_info_[i][IS_REPULSION_FIELD_EDGE]) potential_field_.cells[i].z = 2;

    pub_pf_.publish(potential_field_);
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

    // std::cout<< "plot([";
    // for(int i = 0; i < points.poses.size(); i++)
    // {
    //     std::cout<< points.poses[i].pose.position.x << " ";
    // }
    // std::cout<<"],";
    // std::cout<< "[";
    // for(int i = 0; i < points.poses.size(); i++)
    // {
    //     std::cout<< points.poses[i].pose.position.y << " ";
    // }
    // std::cout<<"])"<< std::endl;

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




    // geometry_msgs::Vector3 end = *points.end();
    // geometry_msgs::Vector3 begin = *points.begin();
    // double xdiff = abs(points[points.size()-1].x - points[0].x);
    // double ydiff = abs(points[points.size()-1].y - points[0].y);
    // ROS_INFO("xdiff:%f     ydiff:%f",xdiff,ydiff);
    // std::vector<geometry_msgs::Vector3> points_original;

    // if (xdiff > ydiff)
    // {
    //     points_original = points;
    // }
    // else
    // {
    //     points_original.resize(points.size());
    //     int i = 0;
    //     for (geometry_msgs::Vector3 p : points)
    //     {
    //         std::cout<< "points =\n" << p <<std::endl;
    //         points_original[i].x = p.y;
    //         points_original[i].y = p.x;
    //         points_original[i].z = p.z;
    //         std::cout<< "points_original =\n" << points_original[i] <<std::endl;
    //         i++;
    //     }
    // }



    // std::vector<geometry_msgs::Vector3> points_original;
    // points_original.resize(points.size());
    // int i = 0;
    // for (geometry_msgs::Vector3 p : points)
    // {
    //     std::cout<< "points =\n" << p <<std::endl;
    //     points_original[i].x = p.y;
    //     points_original[i].y = p.x;
    //     points_original[i].z = p.z;
    //     std::cout<< "points_original =\n" << points_original[i] <<std::endl;
    //     i++;
    // }




    // std::vector<geometry_msgs::Vector3> points_original;
    // points_original.resize(points.size());
    // int size = points.size()-1;
    // for(int i = 0; i < size; i++)
    // {
    //     double xdiff = abs(points[i+1].x - points[i].x);
    //     double ydiff = abs(points[i+1].y - points[i].y);
    //     if (xdiff > ydiff)
    //     {
    //         points_original[i] = points[i];
    //     }
    //     else
    //     {
    //         points_original[i].x = points[i].y;
    //         points_original[i].y = points[i].x;
    //         points_original[i].z = points[i].z;
    //     }
    // }


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

void PathPlanningClass::__create_PathFromCSV()
{
    nav_msgs::Path init;
    robot_path_ = init;

    std::string str_buf;
    std::string str_conma_buf;
    //std::cout<< PATH_PLANNING_FILE <<std::endl;
    std::ifstream ifs_csv_file(PATH_PLANNING_FILE);

    std_msgs::Header hd;
    hd.frame_id = "/" + FRAME_ID_GLOBAL;
    hd.stamp = ros::Time::now();
    robot_path_.header = hd;

    double line_buf[2];
    while (getline(ifs_csv_file, str_buf)) 
    {    
        
        std::istringstream i_stream(str_buf);// 「,」区切りごとにデータを読み込むためにistringstream型にする
        
        int i = 0;
        while (getline(i_stream, str_conma_buf, ',')) // 「,」区切りごとにデータを読み込む
        {
            //std::cout<< str_conma_buf <<std::endl;
            line_buf[i++] = std::stod(str_conma_buf);
        }
        geometry_msgs::PoseStamped point;
        point.pose.position.x = line_buf[0];
        point.pose.position.y = line_buf[1];
        robot_path_.poses.push_back(point);
    }

    static bool goal_published = false;
    if(!goal_published)
    {
        goal_published = true;
        goal_ = robot_path_.poses.back();
        goal_.header = robot_path_.header;
        pub_goal_.publish(goal_);
    }
    
}

bool PathPlanningClass::__PathCollision()
{
    std::vector<nav_msgs::Odometry> obstacles = __get_ObstacleList(2);
    for (int i = 0; i < obstacles.size(); i++)
    {
        geometry_msgs::Pose world_obslacle_pose;
        geometry_msgs::TransformStamped transform;
        static tf2_ros::TransformListener tfListener(tf_buffer_);
        try 
        {
            // ロボット座標系の障害物を世界座標系に変換
            transform = tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, obstacles[i].header.frame_id, ros::Time());
            tf2::doTransform(obstacles[i].pose.pose, world_obslacle_pose, transform);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_ERROR("TF Ereor : %s", ex.what());
            continue;
        }

        for (int j = 0; j < robot_path_.poses.size(); j++)
        {

            if (potbot_lib::utility::get_Distance(world_obslacle_pose.position,robot_path_.poses[j].pose.position) < 0.2)
            {
                return true;
            }
            
        }
    }
    return false;
}

void PathPlanningClass::publishPathPlan()
{
    pub_PP.publish(robot_path_);
}