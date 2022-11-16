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
    manage();
}

void PotentialMethodClass::manage()
{
    odometry();
    if (encoder_first && scan_first)
    {
        //potential();
        cont_pos();
        publishcmd();
        publishodom();
        publishShortestDistance();
    }
}

void PotentialMethodClass::cont_pos()
{
    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

    double error_x_pos = odom.pose.pose.position.x - 6;
    double error_y_pos = odom.pose.pose.position.y - 0;

    double cont_pos_time_now = ros::Time::now().toSec();
    double cont_pos_deltatime = cont_pos_time_now - cont_pos_time_pre;

    double error_x_vel = error_x_pos;
    double error_y_vel = error_y_pos;

    double vel = 0.5 * sqrt(pow(error_x_vel,2) + pow(error_y_vel,2));
    double ang = 0.5 * atan(error_x_vel / error_y_vel);
    
    std::cout<< "vel,ang = " << vel << "," << ang / M_PI * 180 <<std::endl;

    if (vel > 1.0) vel = 1.0;
    if (ang > M_PI_2) ang = M_PI_2;

    cmd.linear.x = vel;
    cmd.angular.z = ang;

    cont_pos_time_pre = cont_pos_time_now;

}

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
		odom.pose.pose.orientation.z += encoder_value.angular.z * encoder_deltatime;
		odom.pose.pose.position.x += encoder_value.linear.x * cos(odom.pose.pose.orientation.z) * encoder_deltatime;
        odom.pose.pose.position.y += encoder_value.linear.x * sin(odom.pose.pose.orientation.z) * encoder_deltatime;

        //std::cout<< ros::WallTime::now() - start_time <<std::endl;
        //std::cout<< encoder_value <<std::endl;
		//std::cout<< odom.pose.pose<<std::endl;

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

geometry_msgs::Vector3 PotentialMethodClass::rho_x()
{
    geometry_msgs::Vector3 ans;
    zero_potential = true;

    // std::vector<float>::iterator minIt = std::min_element(scan.ranges.begin(), scan.ranges.end());
    // size_t minIndex = std::distance(scan.ranges.begin(), minIt);

    // double theta = minIndex * scan.angle_increment;
    
    // std::cout<< "distance = " << *minIt <<std::endl;
    // ans.x = *minIt * sin(theta) + odom.pose.pose.position.x;
    // ans.y = *minIt * cos(theta) + odom.pose.pose.position.y;
    // ans.z = *minIt;



    int size = scan.ranges.size();
    double obs_pos_x_max = 0, obs_pos_x_min = 99999999;
    double obs_pos_y_max = 0, obs_pos_y_min = 99999999;
    double distance;
    //std::cout<< "=======================" <<std::endl;
    for (int i = 0; i < size; i++)
    {
        if (!isinf(scan.ranges[i]))
        {
            double theta = i * scan.angle_increment;
            double obs_x = cos(theta) * scan.ranges[i];
            double obs_y = sin(theta) * scan.ranges[i];
            double robot_x = odom.pose.pose.position.x;
            double robot_y = odom.pose.pose.position.y;

            double obs_pos_x = obs_x + robot_x;
            if (obs_pos_x > obs_pos_x_max) obs_pos_x_max = obs_pos_x;
            if (obs_pos_x < obs_pos_x_min) obs_pos_x_min = obs_pos_x;

            double obs_pos_y = obs_y + robot_y;
            if (obs_pos_y > obs_pos_y_max) obs_pos_y_max = obs_pos_y;
            if (obs_pos_y < obs_pos_y_min) obs_pos_y_min = obs_pos_y;

            //std::cout<< "obs = "<< obs_pos_x << "," << obs_pos_y <<std::endl;
            
        }
        if (zero_potential && scan.ranges[i] <= rho_zero)
        {
            distance = scan.ranges[i];
            zero_potential = false;
        }
    }
    //std::cout<< "=======================" <<std::endl;

    if (!zero_potential)
    {
        ans.x = obs_pos_x_max - obs_pos_x_min;
        ans.y = obs_pos_y_max - obs_pos_y_min;
        
        ans.z = distance;
    }

    // int size = obstacle.size();
    // for (int i=0; i<size; i++)
    // {
    //     if (ans.x <= obstacle[i][0] + 0.05 && ans.x >= obstacle[i][0] - 0.05 &&
    //         ans.y <= obstacle[i][1] + 0.05 && ans.y >= obstacle[i][1] - 0.05)
    //     {
    //         ans.x = obstacle[i][0];
    //         ans.y = obstacle[i][1];
    //         break;
    //     }
    //     else
    //     {
    //         if (obstacle[i][0] == 0.0 && obstacle[i][1] == 0.0)
    //         {
    //             obstacle[i][0] = ans.x;
    //             obstacle[i][1] = ans.y;
    //             break;
    //         }
    //         else
    //         {
    //             if (i >= size - 1)
    //             {
    //                 rotate(obstacle.begin(), obstacle.begin() + 1, obstacle.end());
    //                 obstacle[i][0] = ans.x;
    //                 obstacle[i][1] = ans.y;
    //             }
    //         }
            
    //     }

    // }

    // for (int i=0; i<size; i++)
    // {
    //     std::cout<< obstacle[i][0] << "," << obstacle[i][1] << "\n" <<std::endl;
    // }
    // std::cout<< "=========================="<<std::endl;

    //std::cout<< "rho_x = \n" << ans <<std::endl;

    //theta = atan((3.0 - odom.pose.pose.position.x)/(0.0 - odom.pose.pose.position.y));
    //ans.x = (3.0 - odom.pose.pose.position.x) * sin(theta);
    //ans.y = (0.0 - odom.pose.pose.position.y) * cos(theta);
    //ans.z = sqrt(pow((3.0 - odom.pose.pose.position.x), 2.0) + pow((0.0 - odom.pose.pose.position.y), 2.0));

    //if (ans.x > 0.5) ans.x -= 0.5;
    //if (ans.y > 0.5) ans.y -= 0.5;
    ShortestDistance = ans;
    return ans;
}

geometry_msgs::Vector3 rho_x_pre;
bool first = true;
geometry_msgs::Vector3 PotentialMethodClass::F_o()
{
    
    geometry_msgs::Vector3 rho_x_cur = rho_x();
    //std::cout<< "rho_x_cur = \n" << rho_x_cur <<std::endl;
    
    geometry_msgs::Vector3 ans;
    if (rho_x_cur.z <= rho_zero && !first)
    {
        ans.x = 1.0 * (1/rho_x_cur.x - 1/rho_zero) * (1/pow(rho_x_cur.x, 2.0)) * ((rho_x_cur.x - rho_x_pre.x)/(odom.pose.pose.position.x - odom_pre.pose.pose.position.x));
        ans.y = 1.0 * (1/rho_x_cur.y - 1/rho_zero) * (1/pow(rho_x_cur.y, 2.0)) * ((rho_x_cur.y - rho_x_pre.y)/(odom.pose.pose.position.y - odom_pre.pose.pose.position.y));
    }
    else
    {
        ans.x = 0.0;
        ans.y = 0.0;
        //std::cout<< "=======================" <<std::endl;
    }

    //std::cout<< "======================= " << (rho_x_cur.y - rho_x_pre.y) <<std::endl;
    rho_x_pre.x = rho_x_cur.x;
    rho_x_pre.y = rho_x_cur.y;
    first = false;
    //std::cout<< "F_o ans = \n" << ans <<std::endl;
    
    return ans;
}

geometry_msgs::Vector3 PotentialMethodClass::U_xd()
{
    double k_p = 1.0;
    geometry_msgs::Vector3 ans;
    ans.x = 0.5 * k_p * pow(odom.pose.pose.position.x - TARGET_POSITION_X, 2.0);
    ans.y = 0.5 * k_p * pow(odom.pose.pose.position.y - TARGET_POSITION_Y, 2.0);
    
    return ans;
}

geometry_msgs::Vector3 PotentialMethodClass::U_o()
{
    double eta = 10000;
    
    geometry_msgs::Vector3 rho_x_cur = rho_x();
    //std::cout<< "rho_x_cur = \n" << rho_x_cur <<std::endl;
    
    geometry_msgs::Vector3 ans;
    if (!zero_potential && !first)
    {
        ans.x = 0.5 * eta * pow(1/rho_x_cur.z - 1/rho_zero, 2.0);
        ans.y = 0.5 * eta * pow(1/rho_x_cur.y - 1/rho_zero, 2.0);
    }else
    {
        ans.x = 0.0;
        ans.y = 0.0;
        //std::cout<< "=======================" <<std::endl;
    }

    //std::cout<< "======================= " << (rho_x_cur.y - rho_x_pre.y) <<std::endl;
    rho_x_pre.x = rho_x_cur.x;
    rho_x_pre.y = rho_x_cur.y;
    first = false;
    //std::cout<< "F_o ans = \n" << ans <<std::endl;
    
    return ans;
}

geometry_msgs::Vector3 PotentialMethodClass::U()
{
    geometry_msgs::Vector3 ans;
    geometry_msgs::Vector3 a = U_xd();
    geometry_msgs::Vector3 b = U_o();
    ans.x = a.x + b.x;
    ans.y = a.y + b.y;
    std::cout<< "zero_potential = " << zero_potential <<std::endl;
    std::cout<< "potential = " << sqrt(pow(ans.x,2) + pow(ans.y,2)) <<std::endl;
    return ans;
}

geometry_msgs::Vector3 PotentialMethodClass::F()
{
    geometry_msgs::Vector3 ans;
    geometry_msgs::Vector3 U_cur = U();
    // geometry_msgs::Vector3 a = F_xd();
    // geometry_msgs::Vector3 b = F_o();
    // ans.x = a.x + b.x;
    // ans.y = a.y + b.y;

    ros::Time f_time_now = ros::Time::now();
    double f_deltatime = f_time_now.toSec() - f_time_pre;
    std::cout<< "f_deltatime = " << f_deltatime <<std::endl;
    if (!std::isnan(U_pre.x) && !std::isnan(U_pre.y))
    {
        ans.x = -(U_cur.x - U_pre.x) / (odom.pose.pose.position.x - odom_pre.pose.pose.position.x);
        ans.y = -(U_cur.y - U_pre.y) / (odom.pose.pose.position.y - odom_pre.pose.pose.position.y);
    }
    std::cout<< "F = " << sqrt(pow(ans.x,2) + pow(ans.y,2)) <<std::endl;
    U_pre = U_cur;
    f_time_pre = f_time_now.toSec();

    return ans;
}

double ang, vel, theta_pre = 1000000;
geometry_msgs::Vector3 velocity;
//ポテンシャル法
void PotentialMethodClass::potential()
{

    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

    geometry_msgs::Vector3 velocity = F();
    //std::cout<< "velocity = \n" << velocity <<std::endl;

    // geometry_msgs::Vector3 accel = F();

    // ros::Time time_now = ros::Time::now();
    // double deltatime = time_now.toSec() - potential_time_pre.toSec();
    // //std::cout<< "accel.y = " << accel.y <<std::endl;
    // potential_time_pre = time_now;

    // double tmp = accel.x * deltatime;
    // if (!std::isnan(tmp)) velocity.x += tmp;

    // tmp = accel.y * deltatime;
    // if (!std::isnan(tmp)) velocity.y += tmp;

    // std::cout<< "accel = \n" << accel <<std::endl;


    double theta = atan(velocity.y / (velocity.x + 0.0000001));
    if (velocity.x < 0 && velocity.y > 0) 
	{
		theta += M_PI;
	}
    else if (velocity.x < 0 && velocity.y < 0) 
    {
        //theta += M_PI;
    }
    else if (velocity.x > 0 && velocity.y < 0) 
    {
        //theta += 2*M_PI;
    }

    if (theta_pre != 1000000)
    {
        //ang = (theta - theta_pre) / encoder_deltatime;
        ang = (theta - odom.pose.pose.orientation.z)*1.0;
        //ang = theta;

        vel = 0.2 * (sqrt(pow(velocity.x, 2.0) + pow(velocity.y, 2.0)) - odom.twist.twist.linear.x);
        if (vel > 0.5)
        {
            vel = 0.5;
        }
        // else if (vel < 0.2)
        // {
        //     vel = 0.2;
        // }
        
        cmd.linear.x = vel;
        cmd.angular.z = ang;
    }
    theta_pre = theta;
    
    //if (sqrt(pow(6.0 - odom.pose.pose.position.x, 2.0) + pow(0.0 - odom.pose.pose.position.y, 2.0)) <= 0.1)
    if (odom.pose.pose.position.x > 6.0)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }

    //std::cout<< "x = " << velocity.x <<std::endl;
    //std::cout<< "y = " << velocity.y <<std::endl;
    std::cout<< "vel = " << vel <<std::endl;
    std::cout<< "ang = " << ang/M_PI*180.0 <<std::endl;

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