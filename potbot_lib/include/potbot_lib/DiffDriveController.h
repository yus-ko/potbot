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

            void to_msg(nav_msgs::Odometry& odom_msg);
            void set_msg(const nav_msgs::Odometry& odom_msg);
            void update();
    };

    namespace Controller{
        
        const int PROCESS_STOP = 0;
        const int PROCESS_ROTATE_DECLINATION = 1;
        const int PROCESS_STRAIGHT = 2;
        const int PROCESS_ROTATE_ANGLE = 3;

        class DiffDriveController : public DiffDriveAgent{
            protected:
                double target_x_ = 0.0;
                double target_y_ = 0.0;
                double target_yaw_ = 0.0;

                double gain_p_ = 1.0;
                double gain_i_ = 0.5;
                double gain_d_ = 0.001;

                double stop_margin_angle_ = 0.1;
                double stop_margin_distance_ = 0.03;

                double max_linear_velocity = 1.0;
                double max_angular_velocity = M_PI;

                double error_angle_i_ = 0.0;
                double error_angle_pre_ = nan("");

                double error_distance_i_ = 0.0;
                double error_distance_pre_ = nan("");

                double error_declination_i_ = 0.0;
                double error_declination_pre_ = nan("");

                int process_ = PROCESS_STOP;

            public:
                DiffDriveController(){};
                ~DiffDriveController(){};

                void set_target(double x, double y, double yaw);
                void set_gain(double p, double i, double d);
                void set_margin(double angle, double distance);
                void set_limit(double linear, double angular);

                void pid_control_angle();
                void pid_control_distance();
                void pid_control_declination();
                void pid_control();
            
        };
    }
}


#endif	// _H_DIFFDRIVECONTROLLER_