#include<potbot_rviz/rviz.h>

rvizClass::rvizClass()
{

	setLaunchParam();	// lanchファイルの読み込み
	sub_odom = nhSub.subscribe("position",1000,&rvizClass::odom_callback,this);
	
	pub_marker = nhPub.advertise<visualization_msgs::MarkerArray>("PotentialValue_MarkerArray", 1000);
	pub_path = nhPub.advertise<nav_msgs::Path>("Robot_Trajectory", 1000);
	
}
rvizClass::~rvizClass(){
}
