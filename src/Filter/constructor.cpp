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

	sub_odom_ = nhSub_.subscribe("/potbot/odom",1,&FilterClass::__odom_callback,this);
	sub_obstacle_ = nhSub_.subscribe("/potbot/segments",1,&FilterClass::__obstacle_callback,this);
}
FilterClass::~FilterClass(){
}
