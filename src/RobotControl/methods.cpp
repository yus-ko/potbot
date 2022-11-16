#include<autonomous_mobile_robot_2022/RobotControl.h>

void RobotControlClass::encoder_callback(const geometry_msgs::Twist& msg)
{
    encoder_value = msg;
    manage();
}

void RobotControlClass::encoder_callback_sim(const nav_msgs::Odometry& msg)
{
    encoder_value = msg.twist.twist;
    //std::cout<<"encoder_callback_sim"<<std::endl;
    manage();
}

void RobotControlClass::manage()
{
    odometry();
    pid_control();
    publishcmd();
    publishodom();
}

void RobotControlClass::odometry()
{
    if (encoder_firsttime)
    {
		encoder_time_pre = ros::Time::now();
		encoder_firsttime = false;
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

        std::cout<< ros::WallTime::now() - start_time <<std::endl;
        std::cout<< encoder_value <<std::endl;
		std::cout<< odom.pose.pose<<std::endl;

		encoder_time_pre = encoder_time_now;
    }

    //std::cout<< atan(TARGET_POSITION_Y / TARGET_POSITION_X) <<std::endl;
    if (!done_turn && abs(odom.pose.pose.orientation.z) >= target_angle) done_turn = true;
    if (!done_straight && sqrt(odom.pose.pose.position.x * odom.pose.pose.position.x + odom.pose.pose.position.y * odom.pose.pose.position.y) > sqrt(TARGET_POSITION_X * TARGET_POSITION_X + TARGET_POSITION_Y * TARGET_POSITION_Y)) done_straight = true;
    
}


void RobotControlClass::pid_control()
{

    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

    if (PID_CONTROL)
    {
        if (!done_turn)
        {
            //角速度
            double error_angular = MAX_ANGULAR - encoder_value.angular.z;
            integral_angular_error += error_angular * encoder_deltatime;  //台形近似にするかも

            cmd.angular.z = (GAIN_PROPORTIONAL * error_angular) + (GAIN_INTEGRAL * integral_angular_error) + (GAIN_DIFFERENTIAL * (error_angular - error_angular_pre) / encoder_deltatime);
            if (IS_SIMULATOR && abs(cmd.angular.z) > abs(MAX_ANGULAR)) cmd.angular.z = MAX_ANGULAR;

            error_angular_pre = error_angular;
            return;
        }

        if (!done_straight)
        {
            //速度
            double error_vel = MAX_VELOCITY - encoder_value.linear.x;
            integral_vel_error += error_vel * encoder_deltatime;  //台形近似にするかも

            cmd.linear.x = (GAIN_PROPORTIONAL * error_vel) + (GAIN_INTEGRAL * integral_vel_error) + (GAIN_DIFFERENTIAL * (error_vel - error_vel_pre) / encoder_deltatime);
            if (IS_SIMULATOR && abs(cmd.linear.x) > abs(MAX_VELOCITY)) cmd.linear.x = MAX_VELOCITY;

            error_vel_pre = error_vel;

            if (ANGLE_CORRECTION)
            {
                cmd.angular.z = target_angle - odom.pose.pose.orientation.z;
            }

        }
    }
    else
    {
        if (!done_turn)
        {
            cmd.angular.z = MAX_ANGULAR;
        }
        if (!done_straight)
        {
            cmd.linear.x = MAX_VELOCITY;
        }
    }
    

}

void RobotControlClass::publishcmd()
{
    if (PUBLISH_COMMAND) pub_cmd.publish(cmd);
}
void RobotControlClass::publishodom()
{
    pub_odom.publish(odom);
}