#include<potbot/rviz.h>

rvizClass::rvizClass()
{

	setLaunchParam();	// lanchファイルの読み込み

	sub_PV = nhSub.subscribe("/potbot/PotentialValue",1000,&rvizClass::PotentialValue_callback,this);
	sub_PP = nhSub.subscribe("/potbot/Path",1000,&rvizClass::PathPlan_callback,this);
	sub_odom = nhSub.subscribe("/potbot/odom",1000,&rvizClass::odom_callback,this);
	
	pub_marker = nhPub.advertise<visualization_msgs::MarkerArray>("/potbot/PotentialValue_MarkerArray", 1000);
	pub_marker_pp = nhPub.advertise<visualization_msgs::MarkerArray>("/potbot/PathPlan_MarkerArray", 1000);
	pub_path = nhPub.advertise<nav_msgs::Path>("/potbot/Robot_Path", 1000);
	
}
rvizClass::~rvizClass(){
}
