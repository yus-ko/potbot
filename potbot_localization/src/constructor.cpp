#include<potbot_localization/Localization.h>

LocalizationClass::LocalizationClass()
{
	
	setLaunchParam();	// lanchファイルの読み込み

	if (LOCALIZATION_METHOD == "dead_reckoning")
	{
		localization_method_id_ = potbot_lib::DEAD_RECKONING;
	}
	else if (LOCALIZATION_METHOD == "paticle_filter")
	{
		localization_method_id_ = potbot_lib::PARTICLE_FILTER;
	}

	sub_scan_=nhSub_.subscribe(TOPIC_SCAN,1,&LocalizationClass::__scan_callback,this);
	sub_map_=nhSub_.subscribe("/map",1,&LocalizationClass::map_callback,this);
	sub_inipose_=nhSub_.subscribe("initialpose",1,&LocalizationClass::inipose_callback,this);
	sub_goal_=nhSub_.subscribe("move_base_simple/goal",1,&LocalizationClass::goal_callback,this);
	sub_point_=nhSub_.subscribe("clicked_point",1,&LocalizationClass::point_callback,this);

	// sub_odom_=nhSub_.subscribe(TOPIC_ODOM,1,&LocalizationClass::__odom_callback,this);
		//sub_encoder_=nhSub_.subscribe("/rover_odo",1,&LocalizationClass::encoder_callback,this);
		//sub_encoder_ = nhSub_.subscribe("/encoder",1,&LocalizationClass::beego_encoder_callback,this);
		//pub_odom_= nhPub_.advertise<nav_msgs::Odometry>(TOPIC_ODOM, 1);

	if (localization_method_id_ == potbot_lib::PARTICLE_FILTER)
	{
		pub_particle_ = nhPub_.advertise<visualization_msgs::MarkerArray>("Particle", 1);
	}

	pub_localmap_ = nhPub_.advertise<nav_msgs::OccupancyGrid>("Localmap", 1);
	
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
    if (localization_method_id_ == potbot_lib::PARTICLE_FILTER) set_pose(initial_pose_);
    // pub_odom_.publish(odom_);

	f_ = boost::bind(&LocalizationClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);
	
}
LocalizationClass::~LocalizationClass(){
}
