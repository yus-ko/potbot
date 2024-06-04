#include<potbot_controller/Controller.h>

namespace potbot_controller
{
    Controller::Controller(tf2_ros::Buffer& tf, const std::string& name) : tf_buffer_(tf)
    {
        ros::NodeHandle n("~");
        n.getParam("publish_control_command",   publish_command_);

        sub_odom_			= nhSub_.subscribe("odom",						1,&Controller::__odom_callback,this);
        sub_path_			= nhSub_.subscribe("Path",						1,&Controller::__path_callback,this);
        sub_goal_			= nhSub_.subscribe("goal",						1,&Controller::__goal_callback, this);

        pub_path_request_	= nhPub_.advertise<std_msgs::Empty>(				"create_path", 1);
        pub_cmd_			= nhPub_.advertise<geometry_msgs::Twist>(			"cmd_vel", 1);
        pub_look_ahead_		= nhPub_.advertise<visualization_msgs::Marker>(		"LookAhead", 1);
        
        dsrv_ = new dynamic_reconfigure::Server<potbot_controller::ControllerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<potbot_controller::ControllerConfig>::CallbackType cb = boost::bind(&Controller::__param_callback, this, _1, _2);
        dsrv_->setCallback(cb);

        // nav_server_ = new NavServer(ros::NodeHandle(), "potbot_cont", boost::bind(&Controller::__execute_callback, this, _1), false);
        // nav_server_->start();
    }

    Controller::~Controller(){
    }

    void Controller::__odom_callback(const nav_msgs::Odometry& msg)
    {
        odom_ = msg;
        robot_controller_.set_msg(odom_);
        robot_controller_.deltatime = 1.0/30.0;
        manage();
    }

    void Controller::__path_callback(const nav_msgs::Path& msg)
    {
        ROS_INFO("subscribe path: controller");
        if (msg.header.frame_id != odom_.header.frame_id)
        {
            nav_msgs::Path init;
            robot_path_ = init;
            potbot_lib::utility::get_tf(tf_buffer_, msg, odom_.header.frame_id, robot_path_);
        }
        else
        {
            robot_path_ = msg;
        }
        robot_controller_.set_target_path(robot_path_);
    }

    void Controller::__goal_callback(const geometry_msgs::PoseStamped& msg)
    {
        ROS_INFO("subscribe goal: controller");
        if (msg.header.frame_id != odom_.header.frame_id)
        {
            goal_ = potbot_lib::utility::get_tf(tf_buffer_, msg, odom_.header.frame_id);
        }
        else
        {
            goal_ = msg;
        }
        robot_controller_.set_target(goal_.pose);
        // __publish_path_request();
    }

    void Controller::__execute_callback(const potbot_msgs::NavigationGoalConstPtr& current_goal)
    {
        ros::Rate loop_rate(2);
        // potbot_msgs::NavigationGoalConstPtr current_goal;
        while (ros::ok())
        {
            ROS_INFO("hello");
            if (nav_server_->isNewGoalAvailable())
            {
                // current_goal = nav_server_->acceptNewGoal();
                printf("Update Goal\n");
                potbot_lib::utility::print_Pose(current_goal->goal_pose.pose);
            }
            if (nav_server_->isActive())
            {
                if (nav_server_->isPreemptRequested())
                {
                    nav_server_->setPreempted();
                    printf("Preempt Goal\n");
                }
                else
                {
                    if (false)
                    {
                        nav_server_->setSucceeded();
                        // server.setAborted();
                        printf("Active: publish result \n");
                    }
                    else
                    {
                        double t = ros::Time::now().toSec();
                        
                        potbot_msgs::NavigationFeedback feedback;
                        feedback.odom.pose.pose = potbot_lib::utility::get_Pose(0, 0, 0, 0, 0, t);
                        potbot_lib::utility::print_Pose(feedback.odom.pose.pose);
                        nav_server_->publishFeedback(feedback);
                    }
                }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void Controller::__param_callback(const potbot_controller::ControllerConfig& param, uint32_t level)
    {
        publish_command_             		= param.publish_control_command;
        stop_margin_angle_					= param.stop_margin_angle;
        stop_margin_distance_				= param.stop_margin_distance;
        distance_to_lookahead_point_		= param.distance_to_lookahead_point;
        distance_change_to_pose_alignment_	= param.distance_change_to_pose_alignment; 

        robot_controller_.set_gain(	    param.gain_p,
                                        param.gain_i,
                                        param.gain_d);

        robot_controller_.set_margin(	stop_margin_angle_,
                                        stop_margin_distance_);

        robot_controller_.set_limit(	param.max_linear_velocity,
                                        param.max_angular_velocity);
        
        robot_controller_.set_distance_to_lookahead_point(distance_to_lookahead_point_);

        robot_controller_.set_initialize_pose(param.initialize_pose);
    }

    void Controller::manage()
    {
        controller();
        if (publish_command_) __publishcmd();
    }

    void Controller::controller()
    {

        double distance = potbot_lib::utility::get_Distance(odom_.pose.pose.position, goal_.pose.position);
        double angle = potbot_lib::utility::get_Yaw(goal_.pose.orientation) - potbot_lib::utility::get_Yaw(odom_.pose.pose.orientation);
        // ROS_INFO("%f / %f, %f / %f", distance, stop_margin_distance_, angle, stop_margin_angle_);
        if (distance < stop_margin_distance_ && abs(angle) < stop_margin_angle_)
        {
            ROS_INFO("reach the goal");
            geometry_msgs::Twist cmd;
            cmd_=cmd;
            set_goal_ = false;
        }
        else if (distance < distance_change_to_pose_alignment_)
        {
            if (!set_goal_)
            {
                set_goal_ = true;
                robot_controller_.set_target(goal_.pose);
            }
            __PoseAlignment();
        }
        else
        {   
            if (!robot_path_.poses.empty()) __LineFollowing();
        }
    }

    void Controller::__LineFollowing()
    {
        ROS_INFO("controller: line following");
        robot_controller_.normalized_pure_pursuit();
        // robot_controller_.pure_pursuit();

        nav_msgs::Odometry robot_pose;
        robot_controller_.to_msg(robot_pose);
        
        visualization_msgs::Marker lookahead_msg;
        robot_controller_.get_lookahead(lookahead_msg);
        lookahead_msg.header = odom_.header;

        cmd_ = robot_pose.twist.twist;
        pub_look_ahead_.publish(lookahead_msg);

        if (potbot_lib::utility::get_Distance(odom_.pose.pose.position, robot_path_.poses.back().pose.position) <= distance_to_lookahead_point_-0.01 ||
            potbot_lib::utility::get_Distance(odom_.pose.pose.position, lookahead_msg.pose.position)			>= distance_to_lookahead_point_+1 ) __publish_path_request();
    }

    void Controller::__PoseAlignment()
    {
        ROS_INFO("controller: pose alignment");
        robot_controller_.pid_control();
        nav_msgs::Odometry robot_pose;
        robot_controller_.to_msg(robot_pose);
        cmd_ = robot_pose.twist.twist;
    }

    void Controller::__publishcmd()
    {
        pub_cmd_.publish(cmd_);
    }

    void Controller::__publish_path_request()
    {
        geometry_msgs::PoseStamped goal_init;
        if(goal_init != goal_)
        {
            std_msgs::Empty empty;
            pub_path_request_.publish(empty);
        }
    }
}