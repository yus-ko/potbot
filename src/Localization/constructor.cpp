#include<potbot/Localization.h>

LocalizationClass::LocalizationClass()
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
	else if (ROBOT_NAME == "beego")
	{
		robot_id_ = BEEGO;
	}

	if (LOCALIZATION_METHOD == "dead_reckoning")
	{
		localization_method_id_ = DEAD_RECKONING;
	}
	else if (LOCALIZATION_METHOD == "paticle_filter")
	{
		localization_method_id_ = PARTICLE_FILTER;
	}

	sub_scan_=nhSub_.subscribe("/scan",1,&LocalizationClass::scan_callback,this);
	sub_map_=nhSub_.subscribe("/map",1,&LocalizationClass::map_callback,this);
	sub_inipose_=nhSub_.subscribe("/initialpose",1,&LocalizationClass::inipose_callback,this);
	sub_goal_=nhSub_.subscribe("/move_base_simple/goal",1,&LocalizationClass::goal_callback,this);
	sub_point_=nhSub_.subscribe("/clicked_point",1,&LocalizationClass::point_callback,this);
	sub_cluster_ = nhSub_.subscribe("classificationDataEstimateVelocity", 1, &LocalizationClass::cluster_callback, this);

	if (robot_id_ == MEGAROVER)
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
	else if (robot_id_ == TURTLEBOT3)
	{
		sub_encoder_ = nhSub_.subscribe("odom", 1, &LocalizationClass::encoder_callback_sim, this);
	}
	else if (robot_id_ == BEEGO)
	{
		sub_encoder_ = nhSub_.subscribe("/encoder",1,&LocalizationClass::beego_encoder_callback,this);
	}

	if (localization_method_id_ == PARTICLE_FILTER)
	{
		pub_particle_ = nhPub_.advertise<visualization_msgs::MarkerArray>("Particle", 1);
	}

	pub_localmap_ = nhPub_.advertise<nav_msgs::OccupancyGrid>("Localmap", 1);
	pub_odom_= nhPub_.advertise<nav_msgs::Odometry>("position", 1);
	pub_scan0_ = nhPub_.advertise<sensor_msgs::LaserScan>("scan0", 1);
	pub_scan1_ = nhPub_.advertise<sensor_msgs::LaserScan>("scan1", 1);
	pub_segment_ = nhPub_.advertise<visualization_msgs::MarkerArray>("segment", 1);

	particles_.markers.resize(particle_num_);
	weights_.resize(particle_num_);
	//reset_particle();

	initial_pose_.pose.pose.position.x = INITIAL_POSE_X;
	initial_pose_.pose.pose.position.y = INITIAL_POSE_Y;
	tf2::Quaternion quat;
	quat.setRPY(0, 0, INITIAL_POSE_THETA);
	tf2::convert(quat, initial_pose_.pose.pose.orientation);

	odom_.header = initial_pose_.header;
    odom_.pose = initial_pose_.pose;
    if (localization_method_id_ == PARTICLE_FILTER) set_pose(initial_pose_);
    pub_odom_.publish(odom_);

	f_ = boost::bind(&LocalizationClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);
	
}
LocalizationClass::~LocalizationClass(){
}
