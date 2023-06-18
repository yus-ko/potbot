#include<potbot/Filter.h>
#include <fstream>

FilterClass::FilterClass()
{
	
	setLaunchParam();	// lanchファイルの読み込み

	sub_odom_ = nhSub_.subscribe("position",1,&FilterClass::__odom_callback,this);
	sub_obstacle_ = nhSub_.subscribe("segment",1,&FilterClass::__obstacle_callback,this);
	pub_state_ = nhPub_.advertise<potbot::State>("state", 1);
	
}
FilterClass::~FilterClass(){
}
