#include<potbot_pathplanner/PathPlanning.h>

PathPlanningClass::PathPlanningClass()
{

	//ros::Duration(5).sleep();
	
	setLaunchParam();	// lanchファイルの読み込み
	//__set_Param();

	U_pre.x = std::numeric_limits<double>::quiet_NaN();
	U_pre.y = std::numeric_limits<double>::quiet_NaN();

	obstacle.resize(2);
    obstacle[0].resize(obstacle_size);
    obstacle[1].resize(obstacle_size);
	for (int i = 0; i < obstacle_size; i++)
	{
		obstacle[0][i] = 99999999999;
		obstacle[1][i] = 99999999999;
	}

	// obstacle[0].resize(obstacle_size*1000);
    // obstacle[1].resize(obstacle_size*1000);
	// for (int i = 0; i < obstacle_size*1000; i++)
	// {
	// 	obstacle[0][i] = 99999999999;
	// 	obstacle[1][i] = 99999999999;
	// }

	scan_range_maximum_angle.resize(scan_range_maximum_angle_size);

	if (PATH_PLANNING_METHOD == "csv")
	{
		path_planning_id = potbot_lib::CSV_PATH;
	}
	else if (PATH_PLANNING_METHOD == "potential_method")
	{
		path_planning_id = potbot_lib::POTENTIAL_METHOD;
	}

	sub_scan=nhSub.subscribe("scan",1,&PathPlanningClass::scan_callback,this);

	sub_encoder = nhSub.subscribe("position", 1, &PathPlanningClass::encoder_callback_sim, this);

	if (USE_AMCL) sub_encoder = nhSub.subscribe("/amcl_pose", 1, &PathPlanningClass::pwcs_callback, this);

	pub_goal_ = nhPub.advertise<geometry_msgs::PoseStamped>("goal", 1);
	sub_goal_ = nhSub.subscribe("/move_base_simple/goal", 1, &PathPlanningClass::goal_callback, this);
	if (!USE_RVIZ)
	{
		goal_.pose.position.x = TARGET_POSITION_X;
		goal_.pose.position.y = TARGET_POSITION_Y;
		pub_goal_.publish(goal_);
	}

	sub_odom_ = nhSub.subscribe("position",1,&PathPlanningClass::__odom_callback,this);
	sub_coefficient = nhSub.subscribe("/potential_coefficient", 1, &PathPlanningClass::coefficient_callback, this);
	sub_local_map_ = nhSub.subscribe("Localmap", 1, &PathPlanningClass::local_map_callback, this);
	sub_run_ = nhSub.subscribe("create_path", 1, &PathPlanningClass::__create_path_callback, this);
	sub_seg_ = nhSub.subscribe("segment", 1, &PathPlanningClass::__segment_callback, this);
	sub_state_ = nhSub.subscribe("state",1,&PathPlanningClass::__state_callback,this);
	
	//pub_odom= nhPub.advertise<nav_msgs::Odometry>("/potbot/odom", 1);
	pub_pf_ = nhPub.advertise<nav_msgs::GridCells>("pot", 1);
	pub_PP = nhPub.advertise<nav_msgs::Path>("Path", 1);

	f_ = boost::bind(&PathPlanningClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

}
PathPlanningClass::~PathPlanningClass(){
}
