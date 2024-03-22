#ifndef _H_DIFFDRIVECONTROLLER_
#define _H_DIFFDRIVECONTROLLER_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <potbot_lib/Utility.h>

namespace potbot_lib{

    class DiffDriveAgent{
        public:
            double x                            = 0.0;  //x軸位置 [m]
            double y                            = 0.0;  //y軸位置 [m]
            double yaw                          = 0.0;  //z軸回転 [rad]
            double v                            = 0.0;  //並進速度 [m/s]
            double omega                        = 0.0;  //回転角速度 [rad/s]

            double deltatime                    = 0.02; //単位時間 [s]

            DiffDriveAgent( const double x              = 0.0,
                            const double y              = 0.0,
                            const double yaw            = 0.0,
                            const double v              = 0.0,
                            const double omega          = 0.0,
                            const double deltatime      = 0.02):
                            x(x),
                            y(y),
                            yaw(yaw),
                            v(v),
                            omega(omega),
                            deltatime(deltatime){};

            void to_msg(nav_msgs::Odometry& odom_msg){
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
            };

            void set_msg(const nav_msgs::Odometry& odom_msg){
                x                               = odom_msg.pose.pose.position.x;
                y                               = odom_msg.pose.pose.position.y;
                yaw                             = utility::get_Yaw(odom_msg.pose.pose.orientation);
                v                               = odom_msg.twist.twist.linear.x;
                omega                           = odom_msg.twist.twist.angular.z;
            };

            void update(){
                yaw                             += omega*deltatime;
                x                               += v*deltatime*cos(yaw);
                y                               += v*deltatime*sin(yaw);
            };
    };

    class DiffDriveController : public DiffDriveAgent{
        protected:
            double target_x_ = 0.0;
            double target_y_ = 0.0;
            double target_yaw_ = 0.0;

            double gain_p_ = 1.0;
            double gain_i_ = 0.5;
            double gain_d_ = 0.001;

            double error_angle_i_ = 0.0;
            double error_angle_pre_ = nan("");

            double error_distance_i_ = 0.0;
            double error_distance_pre_ = nan("");

        public:
            DiffDriveController(){};
            ~DiffDriveController(){};
        
            set_gain();

            void pid_controller(){
                if (!isfinite(error_angle_pre_) || !isfinite(error_distance_pre_)) return;

                double error_angle = target_yaw_-yaw;
                error_angle_i_ += error_angle*deltatime;
                double error_angle_d = (error_angle - error_angle_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_angle + gain_i_*error_angle_i_ + gain_d_*error_angle_d;
                error_angle_pre_ = error_angle_d;

                double error_distance = sqrt(pow(target_x_-x,2)+pow(target_y_-y,2));
                error_distance_i_ += error_distance*deltatime;
                double error_distance_d = (error_distance - error_distance_pre_)/deltatime;
                double linear_velocity = gain_p_*error_distance + gain_i_*error_distance_i_ + gain_d_*error_distance_d;
                error_distance_pre_ = error_distance_d;

            };
        
    };
}


#endif	// _H_DIFFDRIVECONTROLLER_