#include<autonomous_mobile_robot_2022/PotentialMethod.h>

void PotentialMethodClass::encoder_callback(const geometry_msgs::Twist& msg)
{
    encoder_value = msg;
    encoder_first = true;
    manage();
}

void PotentialMethodClass::encoder_callback_sim(const nav_msgs::Odometry& msg)
{
    encoder_value = msg.twist.twist;
    encoder_first = true;
    manage();
}

void PotentialMethodClass::scan_callback(const sensor_msgs::LaserScan& msg)
{
    scan = msg;
    scan_first = true;
    //manage();
}

void PotentialMethodClass::manage()
{
    std::cout<< "----------------------------------------" <<std::endl;
    odometry();
    if (encoder_first && scan_first)
    {
        line_following();

        // transform_obstacle_pos();
        // potential();
        // //cont_pos(-3,-3);
        publishcmd();
        publishodom();
        // publishShortestDistance();
        // publishPotentialValue();
    }
}

void PotentialMethodClass::line_following()
{
    //std::cout<< "======================================" <<std::endl;
    geometry_msgs::Vector3 sub_goal = robot_path[robot_path_index];
    int robot_path_size = robot_path.size();
    std::cout<< "robot_path_index = " << robot_path_index <<std::endl;

    double margin = 0.1;
    if (robot_path_index >= robot_path_size - 1) margin = 0.01;

    if (sqrt(pow(odom.pose.pose.position.x - sub_goal.x,2) + pow(odom.pose.pose.position.y - sub_goal.y,2)) <= margin)
    //if (sqrt(pow(odom.pose.pose.position.x - sub_goal.x,2)) <= 0.05)
    {
        robot_path_index++;
    }

    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

    if (robot_path_index < robot_path_size)
    {    
        if (robot_path_index == robot_path_index_pre)
        {
            double y_pr = -sin(sub_start.z) * (odom.pose.pose.position.x - sub_start.x) + cos(sub_start.z) * (odom.pose.pose.position.y - sub_start.y);
            cmd.linear.x = 0.2;
            cmd.angular.z = - 1.0 * -y_pr - 5.0 * (odom.pose.pose.orientation.z - sub_start.z);
        }
        else
        {
            sub_start.x = odom.pose.pose.position.x;
            sub_start.y = odom.pose.pose.position.y;
            sub_start.z = atan2(robot_path[robot_path_index].y - odom.pose.pose.position.y, robot_path[robot_path_index].x - odom.pose.pose.position.x);
        }
    }

    std::cout<< "omega = " << cmd.angular.z /M_PI*180 <<std::endl;

    std::cout<< "sub_start : \n\tx = " << sub_start.x << "\n\ty = " << sub_start.y << "\n\tz = " << sub_start.z /M_PI*180 <<std::endl;
    std::cout<< "sub_goal : \n\tx = " << sub_goal.x << "\n\ty = " << sub_goal.y << "\n\tz = " << sub_start.z /M_PI*180 <<std::endl;

    robot_path_index_pre = robot_path_index;
}

void PotentialMethodClass::cont_vel(double vel_x, double vel_y)
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

void PotentialMethodClass::cont_pos(double goal_x, double goal_y)
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
    //if (target_angle > -0.3 && target_angle < 0.3) target_angle = 0;
    //if (target_angle > M_PI-0.3 && target_angle < -(M_PI-0.3)) target_angle = M_PI;
    //if (target_angle < M_PI) target_angle += 2*M_PI;

    std::cout<< "target_distance = " << target_distance <<std::endl;
    std::cout<< "target_angle = " << target_angle /M_PI*180 <<std::endl;
    std::cout<< "odom_x = " << odom_x <<std::endl;
    std::cout<< "odom_y = " << odom_y <<std::endl;
    std::cout<< "odom_theta = " << odom_theta /M_PI*180 <<std::endl;

    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
    
    cmd.linear.x = 0.2;
    //if (abs(odom_theta - target_angle) > M_PI_4) cmd.linear.x = 0;
    //if (target_distance < 0.14) cmd.linear.x = 0;

    double theta_error = target_angle - odom_theta;

    double cont_pos_time_now = ros::Time::now().toSec();
    if (cont_pos_time_pre > 0) integral_theta_error += integral_theta_error * (cont_pos_time_now - cont_pos_time_pre);
    cont_pos_time_pre = cont_pos_time_now;

    //cmd.angular.z = 0.5*theta_error + 0.25*integral_theta_error;
    cmd.angular.z = theta_error;
    if (abs(theta_error) > M_PI_4) cmd.linear.x = 0;
    //if (cmd.angular.z > M_PI_4/2) cmd.angular.z = M_PI_4/2;
    
    // if (distance_to_obstacle <= 0.5) 
    // {
    //     cmd.linear.x = 0;
    //     cmd.angular.z = M_PI_4;
    // }
    //cmd.linear.x = 0.2;
    //cmd.angular.z = 0;
    std::cout<< "vel,ang = " << cmd.linear.x << "," << cmd.angular.z / M_PI * 180 <<std::endl;

    //if (!done_cont_pos && odom_x >= sub_goal_x*0.8 && odom_x <= sub_goal_x*1.2 && odom_y >= sub_goal_y*0.8 && odom_y <= sub_goal_y*1.2)
    if (!done_cont_pos && abs(odom_x) >= abs(sub_goal_x)*0.8 && abs(odom_y) >= abs(sub_goal_y)*0.8)
    {
        done_cont_pos = true;
    }

}

// void PotentialMethodClass::cont_pos(double goal_x, double goal_y)
// {

//     double odom_x = odom.pose.pose.position.x;
//     double odom_y = odom.pose.pose.position.y;
//     double odom_theta = odom.pose.pose.orientation.z - int(odom.pose.pose.orientation.z/(2*M_PI));
//     if (odom_theta > M_PI) odom_theta -= 2*M_PI;

//     if (done_cont_pos)
//     {
//         start_x = odom_x;
//         start_y = odom_y;
//         start_theta = odom_theta;
//         target_distance = sqrt(pow(goal_x - start_x,2) + pow(goal_y - start_y,2));
//         target_angle = atan2(goal_y - start_y, goal_x - start_x);
//         std::cout<< "target_distance = " << target_distance <<std::endl;
//         std::cout<< "target_angle = " << target_angle /M_PI*180 <<std::endl;

//         cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
//         cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

//         done_cont_pos = false;
        
//     }
//     else
//     {

//         cmd.linear.x = 1.0;

//         if (target_angle >= 0)
//         {
//             cmd.angular.z = 0.2;
//             if (odom_theta >= target_angle)
//             {
//                 cmd.angular.z = 0;
//             }
//             else
//             {
//                 cmd.linear.x = 0;
//             }
//         }
//         else
//         {
//             cmd.angular.z = -0.2;
//             if (odom_theta <= target_angle){
//                 cmd.angular.z = 0;
//             }
//             else
//             {
//                 cmd.linear.x = 0;
//             }
//         }

//         if (sqrt(pow(start_x - odom.pose.pose.position.x,2) + pow(start_y - odom.pose.pose.position.y,2)) >= target_distance)
//         {
//             cmd.linear.x = 0;
//         }

//         if (cmd.linear.x == 0 && cmd.angular.z == 0)
//         {
//             done_cont_pos = true;
//             done_cont_pos_cnt++;
//         }
//     }

    //std::cout<< "vel,ang = " << cmd.linear.x << "," << cmd.angular.z / M_PI * 180 <<std::endl;

    // double goal_x = 0, goal_y = 3;
    // double start_x = 0, start_y = 0;

    // double centar_x = (goal_x + start_x)/2;
    // double centar_y = (goal_y + start_y)/2;
    // std::cout<< "centar_x,centar_y = " << centar_x << "," << centar_y <<std::endl;

    // double radius = sqrt(pow(centar_x - goal_x,2) + pow(centar_y - goal_y,2)) * sqrt(2);
    // std::cout<< "radius = " << radius <<std::endl;

    // double bottom = sqrt(pow(start_x - goal_x,2) + pow(start_y - goal_y,2));
    // std::cout<< "bottom = " << bottom <<std::endl;
    
    // double target_angle = 2 * asin(bottom / (2*radius));

    // cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    // cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

    // double vel = 1.0;
    // double ang = vel / radius;

    // cmd.linear.x = vel;
    // cmd.angular.z = ang;

    // std::cout<< "target_angle = " << target_angle <<std::endl;
    // if (odom.pose.pose.orientation.z > target_angle)
    // {
    //     cmd.linear.x = 0;
    //     cmd.angular.z = 0;
    // }

    // std::cout<< "vel,ang = " << cmd.linear.x << "," << cmd.angular.z / M_PI * 180 <<std::endl;

    // double goal_x = 3;
    // double goal_y = 3;

    // double radius = sqrt(pow(goal_x,2) + pow(goal_y,2));

    // cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    // cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

    // double error_x_pos = goal_x - odom.pose.pose.position.x;
    // double error_y_pos = goal_y - odom.pose.pose.position.y;

    // double cont_pos_time_now = ros::Time::now().toSec();
    // double cont_pos_deltatime = cont_pos_time_now - cont_pos_time_pre;

    // double vel_x = (odom.pose.pose.position.x - pos_x_pre) / cont_pos_deltatime;
    // double vel_y = (odom.pose.pose.position.y - pos_y_pre) / cont_pos_deltatime;

    // double acc_x = (vel_x - vel_x_pre) / cont_pos_deltatime;
    // double acc_y = (vel_y - vel_y_pre) / cont_pos_deltatime;

    // double error_x_vel = error_x_pos;
    // double error_y_vel = error_y_pos;

    // double vel = 0.5 * sqrt(pow(error_x_vel,2) + pow(error_y_vel,2));
    // double ang = 0.5 * atan2(error_y_vel , error_x_vel);
    
    // std::cout<< "vel,ang = " << vel << "," << ang / M_PI * 180 <<std::endl;
    // std::cout<< "theta = " << odom.pose.pose.orientation.z / M_PI * 180 <<std::endl;

    // if (vel > 1.0) vel = 1.0;
    // if (ang > M_PI_2) ang = M_PI_2;

    // vel = 0.8;
    // cmd.linear.x = pow(pow(vel_x,2)+pow(vel_y,2),1.5);
    // cmd.angular.z = abs(vel_x*acc_y - vel_y*acc_x);

    // pos_x_pre = odom.pose.pose.position.x;
    // pos_y_pre = odom.pose.pose.position.y;
    // vel_x_pre = vel_x;
    // vel_y_pre = vel_y;
    // cont_pos_time_pre = cont_pos_time_now;

// }

//自己位置
void PotentialMethodClass::odometry()
{
    if (odometry_firsttime)
    {
		encoder_time_pre = ros::Time::now();
		odometry_firsttime = false;
        odom.twist.twist.linear.x = odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = odom.twist.twist.angular.y = odom.twist.twist.angular.z = 0.0;
        odom.pose.pose.position.x = odom.pose.pose.position.y = odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = odom.pose.pose.orientation.y = odom.pose.pose.orientation.z = 0.0;
        odom.pose.pose.orientation.w = 1.0;
	}
    else
    {
        ros::Time encoder_time_now = ros::Time::now();
		encoder_deltatime = encoder_time_now.toSec() - encoder_time_pre.toSec();

        if (bottom_v == 0 && bottom_omega == 0)
        {
            bottom_v = encoder_value.linear.x;
            bottom_omega = encoder_value.angular.z;
        }

        odom.pose.pose.orientation.z += (bottom_omega + encoder_value.angular.z) * encoder_deltatime / 2;
        double vel = (bottom_v + encoder_value.linear.x) * encoder_deltatime / 2;
        odom.pose.pose.position.x += vel * cos(odom.pose.pose.orientation.z);
        odom.pose.pose.position.y += vel * sin(odom.pose.pose.orientation.z);

        bottom_v = encoder_value.linear.x;
        bottom_omega = encoder_value.angular.z;

        //std::cout<< ros::WallTime::now() - start_time <<std::endl;
        //std::cout<< encoder_value <<std::endl;
		//std::cout<< odom.pose.pose<<std::endl;

        std::cout<< "odom_x     = " << odom.pose.pose.position.x <<std::endl;
        std::cout<< "odom_y     = " << odom.pose.pose.position.y <<std::endl;
        std::cout<< "odom_theta = " << odom.pose.pose.orientation.z /M_PI*180 <<std::endl;

		encoder_time_pre = encoder_time_now;
    }
    //std::cout<< odom.pose.pose.position <<std::endl;
    
}

geometry_msgs::Vector3 PotentialMethodClass::F_xd()
{   
    geometry_msgs::Vector3 ans;
    ans.x = -1.0 * (odom.pose.pose.position.x - TARGET_POSITION_X);
    ans.y = -1.0 * (odom.pose.pose.position.y - TARGET_POSITION_Y);
    
    return ans;
}

void PotentialMethodClass::transform_obstacle_pos()
{
    int size = scan.ranges.size();

    double bias;
    distance_to_obstacle = 9999999999999;
    for (int i = 0; i < size; i++)
    {
        if (!isinf(scan.ranges[i]))
        {
            if (scan.ranges[i] < distance_to_obstacle) distance_to_obstacle = scan.ranges[i];
            if (i > size/2)
            {
                bias = -M_PI_2;
            }
            else
            {
                bias = 3*M_PI_2;
            }
            double theta = i * scan.angle_increment + bias + odom.pose.pose.orientation.z;
            
            obstacle[0][obstacle_index] = cos(theta) * scan.ranges[i] + odom.pose.pose.position.x;
            obstacle[1][obstacle_index] = sin(theta) * scan.ranges[i] + odom.pose.pose.position.y;
            obstacle_index++;
            if (obstacle_index >= obstacle_size) obstacle_index = 0;
        }
    }

}

// void PotentialMethodClass::transform_obstacle_pos()
// {

//     int size = scan.ranges.size();
    
//     obstacle[0].resize(size);
//     obstacle[1].resize(size);

//     int index = 0;
//     for (int i = 0; i < size; i++)
//     {
//         if (!isinf(scan.ranges[i]))
//         {
//             double theta = i * scan.angle_increment;
            
//             obstacle[0][index] = cos(theta) * scan.ranges[i] + odom.pose.pose.position.x;
//             obstacle[1][index] = sin(theta) * scan.ranges[i] + odom.pose.pose.position.y;
//             index++;
//         }
//     }
    
//     obstacle[0].resize(index);
//     obstacle[1].resize(index);

//     // obstacle[0].resize(10000);
//     // obstacle[1].resize(10000);
//     // for (int i = 0; i < 100; i++)
//     // {
//     //     for (int j = 0; j < 100; j++)
//     //     {
//     //         obstacle[0][i*100 + j] = 2.5 + 0.01*i;
//     //         obstacle[1][i*100 + j] = 0 + 0.01*j;
//     //     }
//     // }
    

// }

// void PotentialMethodClass::transform_obstacle_pos()
// {

//     int size = scan.ranges.size();
//     int extend_size = 1000;

//     std::vector<std::vector<float>> extended_scan;
//     extended_scan.resize(2);
//     extended_scan[0].resize(size*extend_size);
//     extended_scan[1].resize(size*extend_size);
    
//     int cnt_notinf = 0;
//     for (int i = 0; i < size; i++)
//     {
//         if (!isinf(scan.ranges[i]))
//         {
//             double theta = i * scan.angle_increment;
//             for (int j = 0; j < extend_size; j++)
//             {
//                 extended_scan[0][cnt_notinf + j] = theta;
//                 extended_scan[1][cnt_notinf + j] = scan.ranges[i] + j * 0.002;
//             }
//             cnt_notinf++;
//         }
//     }
    
//     size = cnt_notinf*extend_size;

//     extended_scan[0].resize(size);
//     extended_scan[1].resize(size);

// 	obstacle[0].resize(size);
//     obstacle[1].resize(size);
    
//     for (int i = 0; i < size; i++)
//     {
//         obstacle[0][i] = cos(extended_scan[0][i]) * extended_scan[1][i] + odom.pose.pose.position.x;
//         obstacle[1][i] = sin(extended_scan[0][i]) * extended_scan[1][i] + odom.pose.pose.position.y;
//     }

// }

// void PotentialMethodClass::transform_obstacle_pos()
// {

//     int size = scan.ranges.size();
//     int extend_size = 1000;

//     std::vector<std::vector<float>> extended_scan;
//     extended_scan.resize(2);
//     extended_scan[0].resize(size*extend_size);
//     extended_scan[1].resize(size*extend_size);
    
//     int cnt_notinf = 0;
//     for (int i = 0; i < size; i++)
//     {
//         if (!isinf(scan.ranges[i]))
//         {
//             double theta = i * scan.angle_increment;
//             for (int j = 0; j < extend_size; j++)
//             {
//                 extended_scan[0][cnt_notinf + j] = theta;
//                 extended_scan[1][cnt_notinf + j] = scan.ranges[i] + j * 0.002;
//             }
//             cnt_notinf++;
//         }
//     }
    
//     size = cnt_notinf*extend_size;

//     extended_scan[0].resize(size);
//     extended_scan[1].resize(size);

// 	//obstacle[0].resize(size);
//     //obstacle[1].resize(size);
    
//     for (int i = 0; i < size; i++)
//     {
//         obstacle[0][obstacle_index] = cos(extended_scan[0][i]) * extended_scan[1][i] + odom.pose.pose.position.x;
//         obstacle[1][obstacle_index] = sin(extended_scan[0][i]) * extended_scan[1][i] + odom.pose.pose.position.y;
//         obstacle_index++;
//         if (obstacle_index >= obstacle_size*1000) obstacle_index = 0;
        
//     }

// }

geometry_msgs::Vector3 PotentialMethodClass::rho_x(double robot_x, double robot_y)
{
    geometry_msgs::Vector3 ans;
    ans.z = 99999999999;

    
    int size = obstacle[0].size();
    //std::cout<< "=======================" <<std::endl;

    double diffx = robot_x - odom.pose.pose.position.x;
    double diffy = robot_y - odom.pose.pose.position.y;
    double angle = atan2(diffy,diffx);
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

        ans.z = sqrt(pow(robot_x - obstacle[0][i],2) + pow(robot_y - obstacle[1][i],2));

        if (ans.z <= rho_zero)
        {
            break;
        }

    }
    
    ShortestDistance = ans;
    return ans;
}

geometry_msgs::Vector3 PotentialMethodClass::U_xd(double robot_x, double robot_y)
{
    double k_p = 1;
    geometry_msgs::Vector3 ans;
    ans.x = 0.5 * k_p * pow(robot_x - TARGET_POSITION_X, 2.0);
    ans.y = 0.5 * k_p * pow(robot_y - TARGET_POSITION_Y, 2.0);
    
    return ans;
}

geometry_msgs::Vector3 PotentialMethodClass::U_o(double robot_x, double robot_y)
{
    double eta = 10000;
    
    bool in_obstacle = false;

    int size = obstacle[0].size();
    //std::cout<< "=======================" <<std::endl;

    // double diffx = robot_x - odom.pose.pose.position.x;
    // double diffy = robot_y - odom.pose.pose.position.y;
    // double angle = atan2(diffy,diffx);
    // for (int i = 0; i < size; i++)
    // {
    //     double obs_x = obstacle[0][i] - odom.pose.pose.position.x;
    //     double obs_y = obstacle[1][i] - odom.pose.pose.position.y;
    //     double obs_angle = atan2(obs_y,obs_x);
    //     if (//angle >= 0.9*obs_angle && angle <= 1.1*obs_angle &&
    //         abs(obs_x) <= abs(diffx) && 
    //         abs(obs_y) <= abs(diffy))
    //     {
    //         in_obstacle = true;
    //         break;
    //     }
    // }

    // double angle = atan2(robot_y - odom.pose.pose.position.y, robot_x - odom.pose.pose.position.x) + M_PI_2;
    // int angle_index = angle / scan.angle_increment;
    // if (angle <= M_PI && angle >= 0 &&
    //     !isinf(scan.ranges[angle_index-1]) &&
    //     !isinf(scan.ranges[angle_index]) &&
    //     !isinf(scan.ranges[angle_index+1]))
    // {
    //     //std::cout<< "angle = " << angle /M_PI*180 <<std::endl;
    //     in_obstacle = true;
    // }

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

geometry_msgs::Vector3 PotentialMethodClass::U(double robot_x, double robot_y)
{
    geometry_msgs::Vector3 ans;
    geometry_msgs::Vector3 a = U_xd(robot_x, robot_y);
    geometry_msgs::Vector3 b = U_o(robot_x, robot_y);
    ans.x = a.x + b.x;
    ans.y = a.y + b.y;
    return ans;
}

geometry_msgs::Vector3 PotentialMethodClass::F()
{
    geometry_msgs::Vector3 ans;

    double width = 1;

    double x_init = odom.pose.pose.position.x - width/2;
    double x_increment = width/10;
    double x_final = odom.pose.pose.position.x + width/2;

    double y_init = odom.pose.pose.position.y - width/2;
    double y_increment = width/10;
    double y_final = odom.pose.pose.position.y + width/2;

    double F_x = 0;
    double F_y = 0;
    double potential_min = 999999999999;

    int size_X = (abs(x_final - x_init) / x_increment) + 1;
    int size_Y = (abs(y_final - y_init) / y_increment) + 1;
    int size = size_X * size_Y;
    // std::cout<< "sizeX = " << size_X <<std::endl;
    // std::cout<< "sizeY = " << size_Y <<std::endl;
    // std::cout<< "size = " << size <<std::endl;

    PV.data.resize(size);
    geometry_msgs::Vector3 init;
    for (int i=0;i<size;i++) PV.data[i] = init;

    int index = 0;

    for (double x = x_init; x <= x_final; x += x_increment)
    {
        for (double y = y_init; y <= y_final; y += y_increment)
        {
            geometry_msgs::Vector3 U_val = U(x,y);
            double potential_val = sqrt(pow(U_val.x,2) + pow(U_val.y,2));

            PV.data[index].x = x;
            PV.data[index].y = y;
            PV.data[index].z = potential_val;

            if (x >= odom.pose.pose.position.x - (x_increment*0.9) && x <= odom.pose.pose.position.x + (x_increment*0.9) &&
                y >= odom.pose.pose.position.y - (y_increment*0.9) && y <= odom.pose.pose.position.y + (y_increment*0.9))
            {
                PV.start_pos = PV.data[index];
            }

            if (potential_val < potential_min) //&& sqrt(pow(x - odom.pose.pose.position.x,2) + pow(y - odom.pose.pose.position.y,2)) < 1)
            //if (x == x_final && y >= odom.pose.pose.position.y - (y_increment*0.9) && y <= odom.pose.pose.position.y + (y_increment*0.9))
            //if (y == y_final && x >= odom.pose.pose.position.x - (x_increment*0.9) && x <= odom.pose.pose.position.x + (x_increment*0.9))
            //if (x == x_init && y >= odom.pose.pose.position.y - (y_increment*0.9) && y <= odom.pose.pose.position.y + (y_increment*0.9))
            //if (y == y_init && x >= odom.pose.pose.position.x - (x_increment*0.9) && x <= odom.pose.pose.position.x + (x_increment*0.9))
            {
                PV.min = PV.data[index];
                potential_min = potential_val;
                F_x = x;
                F_y = y;
            }
            index++;
        }
    }
    std::cout<< "index = " << index-1 <<std::endl;
    
    std::cout<< "potential_min = " << potential_min <<std::endl;
    std::cout<< "x = " << F_x <<std::endl;
    std::cout<< "y = " << F_y <<std::endl;

    ans.x = F_x;
    ans.y = F_y;

    return ans;
}

//ポテンシャル法
void PotentialMethodClass::potential()
{

    // geometry_msgs::Vector3 potential_min_point = F();
    // cmd.linear.x = 0.2;
    // cmd.angular.z = 0.0;

    if (true||done_cont_pos)
    {
        geometry_msgs::Vector3 potential_min_point = F();

        //cont_vel(potential_min_point.x, potential_min_point.y);
        cont_pos(potential_min_point.x, potential_min_point.y);
    }
    else
    {
        cont_pos(0, 0);
    }

    
    if (odom.pose.pose.position.x > 6.0)
    {
        //cmd.linear.x = 0.0;
        //cmd.angular.z = 0.0;
    }

    //std::cout<< "vel, ang = " << cmd.linear.x << "," << cmd.angular.z/M_PI*180.0 <<std::endl;

}

void PotentialMethodClass::publishcmd()
{
    if (PUBLISH_COMMAND) pub_cmd.publish(cmd);
}
void PotentialMethodClass::publishodom()
{
    pub_odom.publish(odom);
    odom_pre = odom;
}
void PotentialMethodClass::publishShortestDistance()
{
    pub_ShortestDistance.publish(ShortestDistance);
}

void PotentialMethodClass::publishPotentialValue()
{
    pub_PV.publish(PV);
}