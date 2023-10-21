#include<potbot_controller/Controller.h>
#include <fstream>

ControllerClass::ControllerClass()
{
	
	setLaunchParam();	// lanchファイルの読み込み

	if (ROBOT_NAME == "megarover")
	{
		robot_id_ = potbot_lib::MEGAROVER;
	}
	else if (ROBOT_NAME == "turtlebot3")
	{
		robot_id_ = potbot_lib::TURTLEBOT3;
	}
	else if (ROBOT_NAME == "beego")
	{
		robot_id_ = potbot_lib::BEEGO;
	}

	sub_odom_ = nhSub_.subscribe("position",1,&ControllerClass::__odom_callback,this);
	sub_path_ = nhSub_.subscribe("Path",1,&ControllerClass::path_callback,this);
	sub_goal_ = nhSub_.subscribe("goal", 1, &ControllerClass::__goal_callback, this);
	sub_local_map_ = nhSub_.subscribe("Localmap", 1, &ControllerClass::__local_map_callback, this);
	sub_scan_ = nhSub_.subscribe("scan",1,&ControllerClass::__scan_callback,this);
	sub_seg_ = nhSub_.subscribe("segment", 1, &ControllerClass::__segment_callback, this);

	pub_path_request_ = nhPub_.advertise<std_msgs::Empty>("create_path", 1);
	
	std::string cmd_topic_name = "invalid_cmd_vel";
	if(PUBLISH_COMMAND)
	{
		if (robot_id_ == potbot_lib::MEGAROVER)
		{
			if(IS_SIMULATOR)
			{
				cmd_topic_name = "/vmegarover/diff_drive_controller/cmd_vel";
			}
			else
			{
				cmd_topic_name = "/rover_twist";
			}
		}
		else if (robot_id_ == potbot_lib::TURTLEBOT3)
		{
			cmd_topic_name = "cmd_vel";
		}
		else if (robot_id_ == potbot_lib::BEEGO)
		{
			cmd_topic_name = "/beego/cmd_vel";
		}
	}
	pub_cmd_ = nhPub_.advertise<geometry_msgs::Twist>(cmd_topic_name, 1);

	f_ = boost::bind(&ControllerClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);
}
ControllerClass::~ControllerClass(){
}
