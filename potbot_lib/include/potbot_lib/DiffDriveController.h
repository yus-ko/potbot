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

            DiffDriveAgent( const double x      = 0.0,
                            const double y      = 0.0,
                            const double yaw    = 0.0,
                            const double v      = 0.0,
                            const double omega  = 0.0):
                            x(x),
                            y(y),
                            yaw(yaw),
                            v(v),
                            omega(omega){};

            void to_msg(nav_msgs::Odometry& odom_msg){
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
    };

    class DiffDriveController : public DiffDriveAgent{
        protected:
        public:
            DiffDriveController(){};
            ~DiffDriveController(){};
        
    };
}


#endif	// _H_DIFFDRIVECONTROLLER_