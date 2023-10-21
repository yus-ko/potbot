#include<potbot_filter/Filter.h>

FilterClass::FilterClass()
{
	
	setLaunchParam();	// lanchファイルの読み込み

    //tf2_ros::TransformListener tfListener(tf_buffer_);

	sub_odom_ = nhSub_.subscribe("position",1,&FilterClass::__odom_callback,this);
	sub_obstacle_ = nhSub_.subscribe("segment",1,&FilterClass::__obstacle_callback,this);
	pub_state_ = nhPub_.advertise<potbot_msgs::StateArray>("state", 1);
	
}
FilterClass::~FilterClass(){
}
