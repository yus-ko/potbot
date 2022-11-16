#include<autonomous_mobile_robot_2022/rviz.h>

rvizClass::rvizClass()
{

	setLaunchParam();	// lanchファイルの読み込み

	sub_PV = nhSub.subscribe("/autonomous_mobile_robot_2022/PotentialValue",1000,&rvizClass::PotentialValue_callback,this);
	sub_PP = nhSub.subscribe("/autonomous_mobile_robot_2022/Path",1000,&rvizClass::PathPlan_callback,this);
	sub_odom = nhSub.subscribe("/autonomous_mobile_robot_2022/odom",1000,&rvizClass::odom_callback,this);
	
	pub_marker = nhPub.advertise<visualization_msgs::MarkerArray>("/autonomous_mobile_robot_2022/PotentialValue_MarkerArray", 1000);
	pub_marker_pp = nhPub.advertise<visualization_msgs::MarkerArray>("/autonomous_mobile_robot_2022/PathPlan_MarkerArray", 1000);
	pub_path = nhPub.advertise<nav_msgs::Path>("/autonomous_mobile_robot_2022/Robot_Path", 1000);
	
}
rvizClass::~rvizClass(){
}
