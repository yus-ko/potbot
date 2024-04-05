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

        void DiffDriveController::set_target_path()
        {
            target_path_.clear();
            for (double x = 0; x <= 2*M_PI; x+=0.01)
            {
                Point p;
                p.x = x;
                p.y = sin(x);
                target_path_.push_back(p);
            }
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
        
        void DiffDriveController::pure_pursuit()
        {
            size_t target_path_size = target_path_.size();
            // double procces = double(robot_path_index_+1)/double(target_path_size);
            // //ROS_INFO("line following processing: %3.1f %% index:%d/%d Done", robot_path_index_, robot_path_size, procces*100);
            // if (procces > 0.9 && potbot_lib::utility::get_Distance(robot_path_.poses[target_path_size-1].pose.position, goal_.pose.position) > 0.3)
            // {
            // }

            // double margin = PATH_TRACKING_MARGIN;

            // //print_Pose(robot_.pose.pose);

            // geometry_msgs::Point sub_goal;
            // double l_d;
            // while(true)
            // {
            //     sub_goal = robot_path_.poses[robot_path_index_].pose.position;
            //     l_d = sqrt(pow(robot_.pose.pose.position.x - sub_goal.x,2) + pow(robot_.pose.pose.position.y - sub_goal.y,2));
            //     if (l_d <= margin)
            //     {
            //         robot_path_index_++;
            //         if (robot_path_index_ >= robot_path_size-1)
            //         {
            //             break;
            //         }
            //     }
            //     else
            //     {
            //         if (l_d > 1.0)
            //         {
            //             __publish_path_request();
            //         }
            //         else
            //         {
            //             visualization_msgs::Marker lookahead;
            //             lookahead.header                = robot_.header;

            //             lookahead.ns                    = "LookAhead";
            //             lookahead.id                    = 0;
            //             lookahead.lifetime              = ros::Duration(0);

            //             lookahead.type                  = visualization_msgs::Marker::SPHERE;
            //             lookahead.action                = visualization_msgs::Marker::MODIFY;
                        
            //             geometry_msgs::PoseStamped pose_in;
            //             pose_in.header                  = robot_path_.header;
            //             pose_in.pose                    = robot_path_.poses[robot_path_index_].pose;
            //             lookahead.pose                  = potbot_lib::utility::get_tf(tf_buffer_, pose_in, FRAME_ID_ROBOT_BASE).pose;

            //             lookahead.scale.x               = 0.03;
            //             lookahead.scale.y               = 0.03;
            //             lookahead.scale.z               = 0.03;

            //             lookahead.color                 = potbot_lib::color::get_msg(potbot_lib::color::RED);
            //             lookahead.color.a               = 0.5;
                        
            //             pub_look_ahead_.publish(lookahead);
            //         }
            //         break;
            //     }
            // }

            // geometry_msgs::Twist cmd;
            // double init_angle, x1,x2,y1,y2;

            // if (robot_path_index_ <= robot_path_size - 2)
            // {
            //     x1 = robot_path_.poses[robot_path_index_].pose.position.x;
            //     x2 = robot_path_.poses[robot_path_index_+1].pose.position.x;
            //     y1 = robot_path_.poses[robot_path_index_].pose.position.y;
            //     y2 = robot_path_.poses[robot_path_index_+1].pose.position.y;
            // }
            // else
            // {
            //     x1 = robot_path_.poses[robot_path_index_-1].pose.position.x;
            //     x2 = robot_path_.poses[robot_path_index_].pose.position.x;
            //     y1 = robot_path_.poses[robot_path_index_-1].pose.position.y;
            //     y2 = robot_path_.poses[robot_path_index_].pose.position.y;
            // }
            // init_angle = atan2(y2-y1,x2-x1);

            // if(!done_init_pose_alignment_ && abs(init_angle - potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation)) > M_PI/6.0)
            // {
            //     geometry_msgs::Quaternion alpha_quat = potbot_lib::utility::get_Quat(0,0,init_angle);
            //     geometry_msgs::Pose target;
            //     target.position = robot_.pose.pose.position;
            //     target.orientation = alpha_quat;
            //     potbot_lib::utility::print_Pose(target);
            //     __PoseAlignment(target);
            // }
            // else if (robot_path_index_ < robot_path_size)
            // {   
            //     done_init_pose_alignment_ = true;
            //     double yaw = potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation);
            //     double alpha = atan2(sub_goal.y - robot_.pose.pose.position.y, sub_goal.x - robot_.pose.pose.position.x) - yaw;
            //     cmd.linear.x = MAX_LINEAR_VELOCITY;
            //     cmd.angular.z = 2*cmd.linear.x*sin(alpha)/l_d;
            //     cmd_ = cmd;
            //     //ROS_INFO("comannd v,omega: %f, %f", cmd_.linear.x, cmd_.angular.z);
            // } 
        }
    }
}