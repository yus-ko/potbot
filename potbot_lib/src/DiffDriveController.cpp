#include <potbot_lib/DiffDriveController.h>

namespace potbot_lib{

    void DiffDriveAgent::to_msg(nav_msgs::Odometry& odom_msg)
    {
        odom_msg.header.stamp           = ros::Time::now();
        odom_msg.pose.pose.position.x   = x;
        odom_msg.pose.pose.position.y   = y;
        odom_msg.pose.pose.position.z   = 0.0;
        odom_msg.pose.pose.orientation  = utility::get_Quat(0,0,yaw);
        odom_msg.twist.twist.linear.x   = v;
        odom_msg.twist.twist.linear.y   = 0.0;
        odom_msg.twist.twist.linear.z   = 0.0;
        odom_msg.twist.twist.angular.x  = 0.0;
        odom_msg.twist.twist.angular.y  = 0.0;
        odom_msg.twist.twist.angular.z  = omega;
    }

    void DiffDriveAgent::set_msg(const nav_msgs::Odometry& odom_msg){
        x                               = odom_msg.pose.pose.position.x;
        y                               = odom_msg.pose.pose.position.y;
        yaw                             = utility::get_Yaw(odom_msg.pose.pose.orientation);
        v                               = odom_msg.twist.twist.linear.x;
        omega                           = odom_msg.twist.twist.angular.z;
    }

    void DiffDriveAgent::update(){
        yaw                             += omega*deltatime;
        x                               += v*deltatime*cos(yaw);
        y                               += v*deltatime*sin(yaw);
    }

    namespace Controller{

        void DiffDriveController::set_target(double x,double y,double yaw){
            process_ = PROCESS_STOP;

            target_x_ = x;
            target_y_ = y;
            target_yaw_ = yaw;

            error_angle_i_ = 0.0;
            error_angle_pre_ = nan("");

            error_distance_i_ = 0.0;
            error_distance_pre_ = nan("");

            error_declination_i_ = 0.0;
            error_declination_pre_ = nan("");
        }

        void DiffDriveController::set_gain(double p,double i,double d){
            gain_p_=p;
            gain_i_=i;
            gain_d_=d;
        }

        void DiffDriveController::set_margin(double angle, double distance)
        {
            stop_margin_angle_ = angle;
            stop_margin_distance_ = distance;
        }

        void DiffDriveController::set_limit(double linear, double angular)
        {
            max_linear_velocity = linear;
            max_angular_velocity = angular;
        }

        void DiffDriveController::pid_control_angle()
        {
            double error_angle = target_yaw_-yaw;
            if (isfinite(error_angle_pre_)){
                error_angle_i_ += error_angle*deltatime;
                double error_angle_d = (error_angle - error_angle_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_angle + gain_i_*error_angle_i_ + gain_d_*error_angle_d;
                omega = alngular_velocity;
            }
            error_angle_pre_ = error_angle;
        }

        void DiffDriveController::pid_control_distance()
        {
            double error_distance = sqrt(pow(target_x_-x,2)+pow(target_y_-y,2));
            if (isfinite(error_distance_pre_)){
                error_distance_i_ += error_distance*deltatime;
                double error_distance_d = (error_distance - error_distance_pre_)/deltatime;
                double linear_velocity = gain_p_*error_distance + gain_i_*error_distance_i_ + gain_d_*error_distance_d;
                v = linear_velocity;
            }
            error_distance_pre_ = error_distance;
        }

        void DiffDriveController::pid_control_declination()
        {
            double error_declination = atan2(target_y_-y,target_x_-x)-yaw;
            if (isfinite(error_declination_pre_)){
                error_declination_i_ += error_declination*deltatime;
                double error_declination_d = (error_declination - error_declination_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_declination + gain_i_*error_declination_i_ + gain_d_*error_declination_d;
                omega = alngular_velocity;
            }
            error_declination_pre_ = error_declination;
        }

        void DiffDriveController::pid_control()
        {
            v=0;
            omega=0;
            
            if (process_ == PROCESS_STOP && (abs(target_yaw_-yaw) >= stop_margin_angle_ || sqrt(pow(target_x_-x,2)+pow(target_y_-y,2)) >= stop_margin_distance_))
            {
                process_ = PROCESS_ROTATE_DECLINATION;
            }

            if (process_ == PROCESS_ROTATE_DECLINATION && abs(atan2(target_y_-y,target_x_-x)-yaw) < stop_margin_angle_)
            {
                process_ = PROCESS_STRAIGHT;
                error_declination_i_ = 0.0;
                error_declination_pre_ = nan("");
            }
            else if (process_ == PROCESS_STRAIGHT && sqrt(pow(target_x_-x,2)+pow(target_y_-y,2)) < stop_margin_distance_)
            {
                process_ = PROCESS_ROTATE_ANGLE;
            }
            else if (process_ == PROCESS_ROTATE_ANGLE && abs(target_yaw_-yaw) < stop_margin_angle_)
            {
                process_ = PROCESS_STOP;
            }
            
            if (process_ == PROCESS_ROTATE_DECLINATION)
            {
                pid_control_declination();
            }
            else if (process_ == PROCESS_STRAIGHT)
            {
                pid_control_declination();
                pid_control_distance();
            }
            else if (process_ == PROCESS_ROTATE_ANGLE)
            {
                pid_control_angle();
            }

            if (v > max_linear_velocity) v = max_linear_velocity;
            else if (v < -max_linear_velocity) v = -max_linear_velocity;
            if (omega > max_angular_velocity) omega = max_angular_velocity;
            else if (omega < -max_angular_velocity) omega = -max_angular_velocity; 
        }
    }
}