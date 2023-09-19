#include<potbot/Controller.h>
#include <fstream>

ControllerClass::ControllerClass()
{
	
	setLaunchParam();	// lanchファイルの読み込み

	if (ROBOT_NAME == "megarover")
	{
		robot_id_ = MEGAROVER;
	}
	else if (ROBOT_NAME == "turtlebot3")
	{
		robot_id_ = TURTLEBOT3;
	}

	sub_odom_ = nhSub_.subscribe("position",1,&ControllerClass::__odom_callback,this);
	sub_path_ = nhSub_.subscribe("Path",1,&ControllerClass::path_callback,this);
	sub_goal_ = nhSub_.subscribe("goal", 1, &ControllerClass::__goal_callback, this);
	sub_local_map_ = nhSub_.subscribe("Localmap", 1, &ControllerClass::__local_map_callback, this);
	sub_scan_ = nhSub_.subscribe("scan",1,&ControllerClass::__scan_callback,this);
	sub_seg_ = nhSub_.subscribe("segment", 1, &ControllerClass::__segment_callback, this);

	pub_path_request_ = nhPub_.advertise<std_msgs::Empty>("create_path", 1);
	

	if(PUBLISH_COMMAND)
	{
		if (robot_id_ == MEGAROVER)
		{
			if(IS_SIMULATOR)
			{
				pub_cmd_ = nhPub_.advertise<geometry_msgs::Twist>("/vmegarover/diff_drive_controller/cmd_vel", 1);
			}
			else
			{
				pub_cmd_ = nhPub_.advertise<geometry_msgs::Twist>("/rover_twist", 1);
			}
		}
		else if (robot_id_ == TURTLEBOT3)
		{
			pub_cmd_ = nhPub_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		}
	}
}
ControllerClass::~ControllerClass(){
}
