#include<potbot/Filter.h>
#include <fstream>

FilterClass::FilterClass()
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

	sub_odom_ = nhSub_.subscribe("position",1,&FilterClass::__odom_callback,this);
	sub_obstacle_ = nhSub_.subscribe("segment",1,&FilterClass::__obstacle_callback,this);
	pub_state_ = nhPub_.advertise<potbot::State>("state", 1);
	
}
FilterClass::~FilterClass(){
}
