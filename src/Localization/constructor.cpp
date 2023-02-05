#include<autonomous_mobile_robot_2022/Localization.h>

LocalizationClass::LocalizationClass()
{
	
	setLaunchParam();	// lanchファイルの読み込み

	if (ROBOT_NAME == "megarover")
	{
		robot_id_ = 0;
	}
	else if (ROBOT_NAME == "turtlebot3")
	{
		robot_id_ = 1;
	}

	sub_scan_=nhSub_.subscribe("/scan",1,&LocalizationClass::scan_callback,this);
	sub_map_=nhSub_.subscribe("/map",1,&LocalizationClass::map_callback,this);
	sub_inipose_=nhSub_.subscribe("/initialpose",1,&LocalizationClass::inipose_callback,this);
	sub_goal_=nhSub_.subscribe("/move_base_simple/goal",1,&LocalizationClass::goal_callback,this);
	sub_point_=nhSub_.subscribe("/clicked_point",1,&LocalizationClass::point_callback,this);

	if (robot_id_ == 0)
	{
		if(IS_SIMULATOR)
		{
			sub_encoder_=nhSub_.subscribe("/vmegarover/diff_drive_controller/odom",1,&LocalizationClass::encoder_callback_sim,this);
		}
		else
		{
			sub_encoder_=nhSub_.subscribe("/rover_odo",1,&LocalizationClass::encoder_callback,this);
		}
	}
	else if (robot_id_ == 1)
	{
		sub_encoder_ = nhSub_.subscribe("/odom", 1, &LocalizationClass::encoder_callback_sim, this);
	}

	pub_particle_ = nhPub_.advertise<visualization_msgs::MarkerArray>("/autonomous_mobile_robot_2022/Particle", 1);
	pub_localmap_ = nhPub_.advertise<nav_msgs::OccupancyGrid>("/autonomous_mobile_robot_2022/Localmap", 1);
	pub_odom_= nhPub_.advertise<nav_msgs::Odometry>("/autonomous_mobile_robot_2022/odom", 1);

	particles_.markers.resize(particle_num_);
	weights_.resize(particle_num_);
	//reset_particle();

	initial_pose_.pose.pose.position.x = INITIAL_POSE_X;
	initial_pose_.pose.pose.position.y = INITIAL_POSE_Y;
	tf2::Quaternion quat;
	quat.setRPY(0, 0, INITIAL_POSE_THETA);
	tf2::convert(quat, initial_pose_.pose.pose.orientation);
	set_pose(initial_pose_);
	
}
LocalizationClass::~LocalizationClass(){
}
