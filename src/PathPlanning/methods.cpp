#include<potbot/PathPlanning.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

void PathPlanningClass::encoder_callback(const geometry_msgs::Twist& msg)
{
    encoder_value = msg;
    encoder_first = true;
    manage();
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
    manage();
}

void PathPlanningClass::encoder_callback_sim(const nav_msgs::Odometry& msg)
{
    header_ = msg.header;
    odom_msg = msg;
    odom = msg;
    encoder_value = msg.twist.twist;
    encoder_first = true;
    manage();
}

void PathPlanningClass::scan_callback(const sensor_msgs::LaserScan& msg)
{
    scan = msg;
    scan_first = true;
    manage();
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
    pub_goal_.publish(goal_);
}

void PathPlanningClass::get_topic()
{
    //時間
    manage_time_pre = manage_time;
    manage_time = ros::Time::now();

	geometry_msgs::TransformStamped state;

    //ロボット位置
	state.header.stamp = manage_time;

	state.header.frame_id = "world";
	state.child_frame_id  = "robot";

    state.transform.translation.x = pwcs_msg.pose.pose.position.x;
    state.transform.translation.y = pwcs_msg.pose.pose.position.y;
    state.transform.translation.z = pwcs_msg.pose.pose.position.z;
    state.transform.rotation = pwcs_msg.pose.pose.orientation;

	robotState_broadcaster.sendTransform(state);
    
    //レーザーレンジファインダー位置
    state.header.frame_id = "robot";
	state.child_frame_id  = "LRF";

	state.transform.translation.x = 0.0;
	state.transform.translation.y = 0.0;
	state.transform.translation.z = 0.0;

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0);
	state.transform.rotation = quat;

    LRF_broadcaster.sendTransform(state);

    //ステレオカメラ位置
    state.header.frame_id = "robot";
	state.child_frame_id  = "StereoCamera";

	state.transform.translation.x = 0.0;
	state.transform.translation.y = 0.0;
	state.transform.translation.z = 0.0;

    quat = tf::createQuaternionMsgFromYaw(0);
	state.transform.rotation = quat;

    StereoCamera_broadcaster.sendTransform(state);

    //レーザーレンジファインダーデータ
    int size = scan_msg.ranges.size();
    sensor_msgs::PointCloud laserPoints_local;
    sensor_msgs::PointCloud laserPoints_world;
    
    laserPoints_local.header.stamp = manage_time;
	laserPoints_local.header.frame_id = "LRF";

    laserPoints_local.points.resize(size);
    
    for (int i = 0; i < size; i++)
    {
        
        double theta = i * scan_msg.angle_increment + scan_msg.angle_min;
        
        laserPoints_local.points[i].x = -sin(theta) * scan_msg.ranges[i];
        laserPoints_local.points[i].y = cos(theta) * scan_msg.ranges[i];
        laserPoints_local.points[i].z = 0;

    }

    // try{
    //     tf.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0));
    //     tf.lookupTransform("/base_link", "/map", ros::Time(0), transform);
    //     std::cout << "transform exist\n";
    // }

    
    std::cout<< "111111111111111111111111111111111111111111" <<std::endl;
    //ros::Duration(5).sleep();
    tflistener.waitForTransform("LRF", "world", manage_time, ros::Duration(3.0));
    tflistener.transformPointCloud("LRF",manage_time,laserPoints_local,"world",laserPoints_world);
    //std::cout<< laserPoints_world.points[540] <<std::endl;

    odom.header.stamp = manage_time;
	odom.header.frame_id = "world";
    odom.pose.pose = pwcs_msg.pose.pose;

    Eigen::Quaternionf quat1(pwcs_msg.pose.pose.orientation.x, pwcs_msg.pose.pose.orientation.y, pwcs_msg.pose.pose.orientation.z, pwcs_msg.pose.pose.orientation.w);

    Eigen::Matrix3f m2=quat1.toRotationMatrix();
	Eigen::Vector3f ea = m2.eulerAngles(1, 2, 0); 
	std::cout<<ea(0)*180.0/M_PI<<",";//roll
	std::cout<<ea(1)*180.0/M_PI<<",";//pitch
	std::cout<<ea(2)*180.0/M_PI<<std::endl;//yaw

    //odom.pose.pose.orientation.z = ea(2);

    // state.transform.translation.y = pwcs_msg.pose.pose.position.y;
    // state.transform.translation.z = pwcs_msg.pose.pose.position.z;
    // state.transform.rotation = pwcs_msg.pose.pose.orientation;
    pub_odom.publish(odom);

}

void PathPlanningClass::manage()
{
    //std::cout<< "----------------------------------------" <<std::endl;
    //get_topic();
    
    if (encoder_first && scan_first)
    {
        transform_obstacle_pos();
        if (path_planning_id == POTENTIAL_METHOD) potential();

        //cont_pos(2,1);

        // cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
        // cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
        // cmd.linear.x = 0.2;
        // publishShortestDistance();
        publishPotentialValue();
        
    }
    odom_pre = odom;
}

void PathPlanningClass::cont_vel(double vel_x, double vel_y)
{
    double vel = sqrt(pow(vel_x,2) + pow(vel_y,2));
    if (distance_to_obstacle <= 0.5) vel = 0;
    if (vel > 0.2) vel = 0.2;
    double ang = atan2(vel_x, vel_y);

    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
    
    cmd.linear.x = vel;
    cmd.angular.z = ang;

    std::cout<< "vel,ang = " << cmd.linear.x << "," << cmd.angular.z / M_PI * 180 <<std::endl;

}

void PathPlanningClass::cont_pos(double goal_x, double goal_y)
{
    //return;
    
    if (done_cont_pos)
    {
        done_cont_pos = false;
        sub_goal_x = goal_x;
        sub_goal_y = goal_y;
    }

    double odom_x = odom.pose.pose.position.x;
    double odom_y = odom.pose.pose.position.y;
    double odom_theta = odom.pose.pose.orientation.z - int(odom.pose.pose.orientation.z/(2*M_PI));
    if (odom_theta > M_PI) odom_theta -= 2*M_PI;

    start_x = odom_x;
    start_y = odom_y;
    start_theta = odom_theta;
    target_distance = sqrt(pow(sub_goal_x - start_x,2) + pow(sub_goal_y - start_y,2));
    target_angle = atan2(sub_goal_y - start_y, sub_goal_x - start_x);

    std::cout<< "target_distance = " << target_distance <<std::endl;
    std::cout<< "target_angle = " << target_angle /M_PI*180 <<std::endl;
    std::cout<< "odom_x = " << odom_x <<std::endl;
    std::cout<< "odom_y = " << odom_y <<std::endl;
    std::cout<< "odom_theta = " << odom_theta /M_PI*180 <<std::endl;

    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
    
    //cmd.linear.x = 0.2;
    cmd.linear.x = sub_goal_x - start_x;
    //cmd.linear.x = target_distance - sqrt(pow(start_x,2) + pow(start_y,2));

    double theta_error = target_angle - odom_theta;

    double cont_pos_time_now = ros::Time::now().toSec();
    if (cont_pos_time_pre > 0) integral_theta_error += integral_theta_error * (cont_pos_time_now - cont_pos_time_pre);
    cont_pos_time_pre = cont_pos_time_now;

    cmd.angular.z = theta_error;
    if (abs(theta_error) > M_PI_4) cmd.linear.x = 0;
    std::cout<< "vel,ang = " << cmd.linear.x << "," << cmd.angular.z / M_PI * 180 <<std::endl;

    //if (!done_cont_pos && odom_x >= sub_goal_x*0.8 && odom_x <= sub_goal_x*1.2 && odom_y >= sub_goal_y*0.8 && odom_y <= sub_goal_y*1.2)
    if (!done_cont_pos && abs(odom_x) >= abs(sub_goal_x)*0.8 && abs(odom_y) >= abs(sub_goal_y)*0.8)
    {
        done_cont_pos = true;
    }

}

geometry_msgs::Vector3 PathPlanningClass::F_xd()
{   
    geometry_msgs::Vector3 ans;
    ans.x = -1.0 * (odom.pose.pose.position.x - goal_.pose.position.x);
    ans.y = -1.0 * (odom.pose.pose.position.y - goal_.pose.position.y);
    
    return ans;
}

void PathPlanningClass::transform_obstacle_pos()
{

    int size = scan.ranges.size();
    double maximum = 0;
    double maximum_angle;
    double diff_angle_min = 99999999;
    double angle_sensor = 0;
    double angle_to_goal = atan2(goal_.pose.position.y - odom.pose.pose.position.y,goal_.pose.position.x - odom.pose.pose.position.x);

    for (int i = 0; i < size; i++)
    {

        double angle = i * scan.angle_increment + scan.angle_min + odom.pose.pose.orientation.z;
        double distance = scan.ranges[i] + scan.range_min;

        if (!isinf(scan.ranges[i]) && distance >= EXCLUDE_LRF)
        {
            

            obstacle[0][obstacle_index] = cos(angle) * distance + odom.pose.pose.position.x;
            obstacle[1][obstacle_index] = sin(angle) * distance + odom.pose.pose.position.y;
            obstacle_index++;
            if (obstacle_index >= obstacle_size) obstacle_index = 0;

            //何もない場所(角度)を取得
            if (scan.ranges[i] > maximum)
            {
                maximum = scan.ranges[i];
                maximum_angle = i * scan.angle_increment + scan.angle_min;
            }
        }

        double tmp_angle = abs(angle - angle_to_goal);
        if (scan.ranges[i] >= 2.5 && tmp_angle < diff_angle_min)
        {
            diff_angle_min = tmp_angle;
            angle_sensor = angle - odom.pose.pose.orientation.z;
        }

        
    }

    coe_0 = 0.9*angle_sensor + 0.05*odom.pose.pose.orientation.z;

    //移動平均
    scan_range_maximum_angle[scan_range_maximum_angle_index++] = maximum_angle;
    if (scan_range_maximum_angle_index >= scan_range_maximum_angle_size)
    {
        moving_average = true;
        scan_range_maximum_angle_index = 0;
    }
    if (moving_average)
    {
        double sum = 0;
        for (int i=0;i<scan_range_maximum_angle_size;i++)
        {
            sum += scan_range_maximum_angle[i];
        }
        //coe_0 = sum/double(scan_range_maximum_angle_size);
    }
    else
    {
        //coe_0 = maximum_angle;
    }

    //ステレオカメラ点群
    int cluster_num = pcl_cluster.data.size();
    //std::cout<< "cluster data size = " << cluster_num <<std::endl;
    for(int i = 0; i < cluster_num; i++)
    {
        double gcx = pcl_cluster.data[i].gc.x;
        double gcy = pcl_cluster.data[i].gc.y;
        if (sqrt(pow(gcx,2.0) + pow(gcy,2.0)) >= 0.05)
        {
            //std::cout<< i << " (x,y) = (" << gcx << ", " << gcy<< ")" <<std::endl;

            obstacle[0][obstacle_index] = gcy + odom.pose.pose.position.x;
            obstacle[1][obstacle_index] = gcx + odom.pose.pose.position.y;
            obstacle_index++;
            if (obstacle_index >= obstacle_size) obstacle_index = 0;
        }
    }

}

// void PathPlanningClass::transform_obstacle_pos()
// {
//     obstacle_index = 0;
//     for (double x = 2; x <= 3; x += 0.025)
//     {
//         for (double y = -0.5; y <= 0.5; y += 0.025)
//         {
//             obstacle[0][obstacle_index] = x;
//             obstacle[1][obstacle_index++] = y;
//         }
//     }

// }

geometry_msgs::Vector3 PathPlanningClass::rho_x(double robot_x, double robot_y)
{
    geometry_msgs::Vector3 ans;
    ans.z = 99999999999;

    
    int size = obstacle[0].size();
    //std::cout<< "=======================" <<std::endl;

    double diffx = robot_x - odom.pose.pose.position.x;
    double diffy = robot_y - odom.pose.pose.position.y;
    double angle = atan2(diffy,diffx);
    double min = 99999999999999;
    for (int i = 0; i < size; i++)
    {

        double obs_x = obstacle[0][i] - odom.pose.pose.position.x;
        double obs_y = obstacle[1][i] - odom.pose.pose.position.y;
        double obs_angle = atan2(obs_y,obs_x);
        if (angle >= 0.9*obs_angle && angle <= 1.1*obs_angle &&
            abs(obs_x) <= abs(diffx) && 
            abs(obs_y) <= abs(diffy))
        {
            ans.z = 0.00000001;
            break;
        }

        double rho_val = sqrt(pow(robot_x - obstacle[0][i],2) + pow(robot_y - obstacle[1][i],2));

        if (rho_val < min) min = rho_val;

    }
    ans.z = min;
    ShortestDistance = ans;
    return ans;
}

geometry_msgs::Vector3 PathPlanningClass::U_xd(double robot_x, double robot_y)
{
    double k_p = 1;
    geometry_msgs::Vector3 ans;
    ans.x = 0.5 * k_p * pow(robot_x - goal_.pose.position.x, 2.0);
    ans.y = 0.5 * k_p * pow(robot_y - goal_.pose.position.y, 2.0);
    
    return ans;
}

geometry_msgs::Vector3 PathPlanningClass::U_o(double robot_x, double robot_y)
{
    double eta = 10000;
    bool in_obstacle = false;
    int size = obstacle[0].size();

    geometry_msgs::Vector3 rho_x_cur = rho_x(robot_x, robot_y);
    //std::cout<< "rho_x_cur = \n" << rho_x_cur <<std::endl;
    
    geometry_msgs::Vector3 ans;
    if (rho_x_cur.z <= rho_zero || in_obstacle)
    {
        ans.x = ans.y = 0.5 * eta * pow(1/rho_x_cur.z - 1/rho_zero, 2.0);
    }else
    {
        ans.x = ans.y = 0.0;
    }
    
    return ans;
}

geometry_msgs::Vector3 PathPlanningClass::U(double robot_x, double robot_y)
{
    geometry_msgs::Vector3 ans;
    geometry_msgs::Vector3 a = U_xd(robot_x, robot_y);
    geometry_msgs::Vector3 b = U_o(robot_x, robot_y);
    ans.x = a.x + b.x;
    ans.y = a.y + b.y;
    return ans;
}

geometry_msgs::Vector3 PathPlanningClass::F()
{
    geometry_msgs::Vector3 ans;

    double width = POTENTIAL_FIELD_WIDTH;
    double div_x = POTENTIAL_FIELD_DIVDE_X;
    double div_y = POTENTIAL_FIELD_DIVDE_Y;

    double x_min = odom.pose.pose.position.x - width/2;
    double x_increment = width/div_x;
    double x_max = odom.pose.pose.position.x + width/2;
    int rows = (x_max - x_min) / x_increment;

    double y_min = odom.pose.pose.position.y - width/2;
    double y_increment = width/div_y;
    double y_max = odom.pose.pose.position.y + width/2;
    int cols = (y_max - y_min) / y_increment;

    PV.x_min = x_min;
    PV.x_increment = x_increment;
    PV.x_max = x_max;
    PV.rows = rows;

    PV.y_min = y_min;
    PV.y_increment = y_increment;
    PV.y_max = y_max;
    PV.cols = cols;

    double F_x = 0;
    double F_y = 0;
    double potential_min = 999999999999;

    int mapsize = rows * cols;

    PV.potential_value.resize(mapsize);
    
    // geometry_msgs::Vector3 U_val_coe = U(odom.pose.pose.position.x, odom.pose.pose.position.y);
    // coe_0 = atan2(U_val_coe.y,U_val_coe.x);

    // std::vector<std::vector<double>> wall;
    // wall.resize(10);

    // double div = 4.0 / 10.0;
    // for (int i = 0; i < obstacle_size; i++)
    // {
    //     if (obstacle[0][i] < odom.pose.pose.position.x + 1.0 && obstacle[0][i] > odom.pose.pose.position.x - 1.0)
    //     {
    //         for (int j = 0; j < 10; j++)
    //         {
    //             if (obstacle[1][i] < odom.pose.pose.position.y + (div * double(j) + 0.2) && 
    //                 obstacle[1][i] > odom.pose.pose.position.y + (div * double(j) - 0.2))
    //             {
    //                 wall[j].push_back(obstacle[1][i]);
    //             }
    //         }
    //     }
        
    // }

    // coe_0 = 0;
    // wall_exists = false;
    // for (int j = 0; j < 10; j++)
    // {
    //     if(wall[j].size() > 500)
    //     {
    //         wall_exists = true;
    //         coe_0 = M_PI_4;
    //         std::cout<< "wall_exists" <<std::endl;
    //         break;
    //     }
    // }

    //std::cout<< "coe_0 = " << coe_0 << "[rad] "<< coe_0/M_PI*180 << "[deg]"<<std::endl;
    for (int cellnum=0;cellnum<mapsize;cellnum++)
    {
        double x = cellnum/cols * x_increment + x_min;
        double y = cellnum%cols * y_increment + y_min;
        geometry_msgs::Vector3 U_val = U(x, y);
        double potential_val = sqrt(pow(U_val.x,2) + pow(U_val.y,2));

        // coe_x = POTENTIAL_BIAS_COEFFICIENT_0;
        // coe_y = POTENTIAL_BIAS_COEFFICIENT_1;
        
        coe_x = cos(coe_0 + M_PI);
        coe_y = sin(coe_0 + M_PI);

        //std::cout<< "coe_x =" << coe_x << "coe_y =" << coe_y <<std::endl;

        double bias = 1;
        if (CALCULATE_BIAS) bias = (coe_x * (x - odom.pose.pose.position.x)) + (coe_y * (y - odom.pose.pose.position.y)) + 1;

        PV.potential_value[cellnum] = potential_val * bias;

        if (x >= odom.pose.pose.position.x - (x_increment*0.9) && x <= odom.pose.pose.position.x + (x_increment*0.9) &&
            y >= odom.pose.pose.position.y - (y_increment*0.9) && y <= odom.pose.pose.position.y + (y_increment*0.9))
        {
            PV.robot_position = cellnum;
        }

        if (PV.potential_value[cellnum] < potential_min)
        {
            PV.min_potential_position = cellnum;
            potential_min = PV.potential_value[cellnum];
            F_x = x;
            F_y = y;
        }
        
    }
    
    // std::cout<< "potential_min = " << potential_min <<std::endl;
    // std::cout<< "x = " << F_x <<std::endl;
    // std::cout<< "y = " << F_y <<std::endl;

    ans.x = 0.5*F_x;
    ans.y = 0.5*F_y;

    return ans;
}

// void PathPlanningClass::path_planning()
// {
//     robot_path.resize(1);
//     robot_path[0].x = PV.min_potential_position/PV.cols * PV.x_increment + PV.x_min;
//     robot_path[0].y = PV.min_potential_position%PV.cols * PV.y_increment + PV.y_min;

//     PP.data.resize(robot_path.size());
//     PP.data = robot_path;
    
// }

void PathPlanningClass::create_exploration_idx(int width)
{
    int idx = 0;
    exploration_arr.resize(idx);

    for (int i=width; i>0; i--)
    {
        exploration_arr.resize(idx+1);
        exploration_arr[idx++] = -PV.cols * width - i;
    }

    for (int i=1; i<=width; i++)
    {
        exploration_arr.resize(idx+1);
        exploration_arr[idx++] = -PV.cols * width + i;
    }

    exploration_arr.resize(idx+1);
    exploration_arr[idx++] = width;

    for (int i=width; i>-1; i--)
    {
        exploration_arr.resize(idx+1);
        exploration_arr[idx++] = PV.cols * width + i;
    }

    for (int i=width; i>0; i--)
    {
        exploration_arr.resize(idx+1);
        exploration_arr[idx++] = PV.cols * width - i;
    }

    exploration_arr.resize(idx+1);
    exploration_arr[idx++] = -width;
}

bool in(int num,std::vector<int> vec)
{
    int size = vec.size();
    for (int i=0;i<size;i++)
    {
        if (vec[i] == num) return true;
    }
    return false;
}

double nCr(double n, double r)
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

void bezier(nav_msgs::Path& points)
{
    // https://www.f.waseda.jp/moriya/PUBLIC_HTML/education/classes/infomath6/applet/fractal/spline/

    std::cout<< "plot([";
    for(int i = 0; i < points.poses.size(); i++)
    {
        std::cout<< points.poses[i].pose.position.x << " ";
    }
    std::cout<<"],";
    std::cout<< "[";
    for(int i = 0; i < points.poses.size(); i++)
    {
        std::cout<< points.poses[i].pose.position.y << " ";
    }
    std::cout<<"])"<< std::endl;


    nav_msgs::Path points_original = points;
    int n = points_original.poses.size();

    for (int i = 0; i < n; i++)
    {
        points.poses[i].pose.position.x = points.poses[i].pose.position.y = 0.0;
    }

    int bezier_idx = 0;
    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
        points.poses.resize(bezier_idx+1);
        for (double i = 0.0; i <= n-1.0; i++)
        {
            
            points.poses[bezier_idx].pose.position.x += nCr(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * points_original.poses[int(i)].pose.position.x;
            points.poses[bezier_idx].pose.position.y += nCr(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * points_original.poses[int(i)].pose.position.y;
        }
        //std::cout<< bezier_idx << " bezier = (" << points[bezier_idx].x << ", " << points[bezier_idx].y << ")" <<std::endl;
        bezier_idx++;
    }
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

void PathPlanningClass::path_planning()
{
    robot_path.header = header_;
    nav_msgs::Path robot_path_tmp = robot_path;

    int pathindex = 0;
    // std::vector<int> exploration_arr = {-PV.cols-1, -PV.cols, -PV.cols+1, 1, PV.cols+1, PV.cols, PV.cols-1, -1,-PV.cols-1,
    //                                     -PV.cols*2-2, -PV.cols*2-1, -PV.cols*2+1,-PV.cols*2+2, 2, PV.cols*2+2, PV.cols*2+1, PV.cols*2, PV.cols*2-2, PV.cols*2-1, -2};
    
    int exploration_size = exploration_arr.size();
    int idx,p_min_idx,cnt=0,cnt_max=sqrt(pow(PV.cols,2)+pow(PV.rows,2))/2;
    int center = PV.robot_position;
    double x_robot = center/PV.cols * PV.x_increment + PV.x_min;
    double y_robot = center%PV.cols * PV.y_increment + PV.y_min;
    centered.resize(1);
    centered[0] = {center};
    double x_pre = std::numeric_limits<double>::infinity();
    double y_pre = 0;

    while (true)
    {
        double x = center/PV.cols * PV.x_increment + PV.x_min;
        double y = center%PV.cols * PV.y_increment + PV.y_min;
        // if (sqrt(pow(odom.pose.pose.position.x - x,2) + pow(odom.pose.pose.position.y - y,2)) < 0.2)
        // {
        //     center++;
        //     continue;
        // }

        if (x <= PV.x_min || x >= PV.x_max || y <= PV.y_min || y >= PV.y_max || cnt > 5) break;

        
        int width = 2;
        bool exist_p_min = false;
        while (width == 2)
        //while (p_min > 1000)
        {
            create_exploration_idx(width++);
            double p_min = std::numeric_limits<double>::infinity();
            for (int i=0; i<exploration_arr.size(); i++)
            {
                idx = center+exploration_arr[i];
                if (!in(idx, centered))
                {
                    double x_tmp = idx/PV.cols * PV.x_increment + PV.x_min;
                    double y_tmp = idx%PV.cols * PV.y_increment + PV.y_min;
                    double positiondiff = sqrt(pow(goal_.pose.position.x - x_tmp,2) + pow(goal_.pose.position.y - y_tmp,2));
                    double posediff = abs(atan2(y_tmp-y_pre,x_tmp-x_pre) - odom.pose.pose.orientation.z);
                    double J = 0.1*PV.potential_value[idx] + positiondiff + posediff;

                    if (J < p_min) 
                    {
                        exist_p_min = true;
                        p_min_idx = idx;
                        p_min = J;
                    }
                }
                
            }
        }
        
        if (!in(p_min_idx, centered) && exist_p_min)
        {
            robot_path.poses.resize(pathindex+1);
            double x_tmp = p_min_idx/PV.cols * PV.x_increment + PV.x_min;
            double y_tmp = p_min_idx%PV.cols * PV.y_increment + PV.y_min;
            robot_path.poses[pathindex].pose.position.x   = x_tmp;
            robot_path.poses[pathindex++].pose.position.y = y_tmp;
            double x_pre = x_tmp;
            double y_pre = y_tmp;
            
            center = p_min_idx;
            std::cout<< center << ", ";
            centered.resize(centered.size()+1);
            centered[centered.size()-1] = center;
            cnt++;
        }
        else
        {
            break;
        }
    }
    std::cout<<std::endl;
    
    int i = 1;
    double th = sqrt(pow(0.25,2)+pow(0.25,2));
    while(i < robot_path.poses.size())
    {
        if (sqrt(pow(robot_path.poses[i].pose.position.x - robot_path.poses[i-1].pose.position.x,2) +
                 pow(robot_path.poses[i].pose.position.y - robot_path.poses[i-1].pose.position.y,2)) > th)
        {
            for (int j = i; j < robot_path.poses.size()-1; j++)
            {
                robot_path.poses[j] = robot_path.poses[j+1];
            }
            robot_path.poses.resize(robot_path.poses.size()-1);
            i = 1;
        }
        else
        {
            i++;
        }
    }

    int pathsize = robot_path_tmp.poses.size();
    if (pathsize > 0)
    {
        double path_score = 0;
        int num = 3;
        for(int i=0; i<num; i++)
        {
            path_score += sqrt(pow(robot_path_tmp.poses[i].pose.position.x - robot_path.poses[i].pose.position.x,2) +
                                pow(robot_path_tmp.poses[i].pose.position.y - robot_path.poses[i].pose.position.y,2));
        }
        path_score=path_score/double(num);
        std::cout<< "path score = " << path_score <<std::endl;

        if (false&&robot_path.poses.size() < 4)
        {
            robot_path.poses.resize(robot_path_tmp.poses.size());
            robot_path = robot_path_tmp;
        }
    }
    
    //延長経路
    // if (pathsize < 3 && pathsize > 1)
    // {
    //     double startx = robot_path[pathsize-1].x;
    //     double starty = robot_path[pathsize-1].y;
    //     double angle = atan2(robot_path[pathsize-1].y - robot_path[pathsize-2].y, robot_path[pathsize-1].x - robot_path[pathsize-2].x);
    //     robot_path.resize(pathsize + 5);
    //     for (int i=0;i<5;i++)
    //     {
    //         robot_path[pathsize+i].x = 0.2*(i+1)*cos(angle) + startx;
    //         robot_path[pathsize+i].y = 0.2*(i+1)*sin(angle) + starty;
    //     }
    // }
    
    //if (sqrt(pow(TARGET_POSITION_X - x_robot,2) + pow(TARGET_POSITION_Y - y_robot,2)) > 1) bezier(robot_path);
    bezier(robot_path);

    // bool use_spline = true;
    // for(int i = 1; i < robot_path.size(); i++)
    // {
    //     if (robot_path[i].x == robot_path[i-1].x)
    //     {
    //         use_spline = false;
    //         break;
    //     }
    // }
    // if (use_spline) spline(robot_path);
    
    double angle_sum = 0;
    for (int i = 0; i < robot_path.poses.size()-1; i++)
    {
        angle_sum += atan2(robot_path.poses[i+1].pose.position.x - robot_path.poses[i].pose.position.x, 
                            robot_path.poses[i+1].pose.position.y - robot_path.poses[i].pose.position.y);
    }

    //coe_0 = angle_sum / double(robot_path.size()-1);

    //std::cout<< "coe_0 = " << coe_0 << "[rad] "<< coe_0/M_PI*180 << "[deg]"<<std::endl;

    //if (robot_path.size() >= 3) spline(robot_path);

    PV.path_plan.resize(centered.size());
    PV.path_plan = centered;
    
}

//ポテンシャル法
void PathPlanningClass::potential()
{

    // geometry_msgs::Vector3 potential_min_point = F();
    // cmd.linear.x = 0.2;
    // cmd.angular.z = 0.0;

    if (true||done_cont_pos)
    {
        geometry_msgs::Vector3 potential_min_point = F();

        //cont_vel(potential_min_point.x, potential_min_point.y);
        //cont_pos(potential_min_point.x, potential_min_point.y);

        double period = PATH_CREATE_PERIOD;
        geometry_msgs::Vector3 ans = U_o(odom.pose.pose.position.x, odom.pose.pose.position.y);
        //std::cout<< "Uo = " << sqrt(pow(ans.x,2) + pow(ans.y,2)) <<std::endl;

        //if(sqrt(pow(ans.x,2) + pow(ans.y,2)) != 0.0) period = 0.2;

        ros::Time nt = ros::Time::now();
        //if ((path_update && (nt.toSec() > path_update_timestamp.toSec() + period || robot_path_index >= robot_path.size())) || wall_exists)
        if (nt.toSec() > path_update_timestamp.toSec() + period)
        //if (path_update)
        {
            // robot_path.resize(1);
            path_update_timestamp = nt;
            // robot_path[0] = potential_min_point;
            path_planning();
            if (sqrt(pow(goal_.pose.position.x - odom.pose.pose.position.x,2) + pow(goal_.pose.position.y - odom.pose.pose.position.y,2)) > TARGET_POSITION_MARGIN)
                publishPathPlan();
        }
        
    }
    else
    {
        cont_pos(0, 0);
    }

    //std::cout<< "vel, ang = " << cmd.linear.x << "," << cmd.angular.z/M_PI*180.0 <<std::endl;

}

void PathPlanningClass::publishShortestDistance()
{
    pub_ShortestDistance.publish(ShortestDistance);
}
void PathPlanningClass::publishPotentialValue()
{
    pub_PV.publish(PV);
}
void PathPlanningClass::publishPathPlan()
{
    pub_PP.publish(robot_path);
}