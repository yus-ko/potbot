#include<potbot/PathPlanning.h>
#include <fstream>

PathPlanningClass::PathPlanningClass()
{

	//ros::Duration(5).sleep();
	
	setLaunchParam();	// lanchファイルの読み込み

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
		path_planning_id = CSV_PATH;
	}
	else if (PATH_PLANNING_METHOD == "potential_method")
	{
		path_planning_id = POTENTIAL_METHOD;
	}

	sub_scan=nhSub.subscribe("scan",1,&PathPlanningClass::scan_callback,this);

	sub_encoder = nhSub.subscribe("/odom", 1, &PathPlanningClass::encoder_callback_sim, this);

	if (USE_AMCL) sub_encoder = nhSub.subscribe("/amcl_pose", 1, &PathPlanningClass::pwcs_callback, this);

	pub_goal_ = nhPub.advertise<geometry_msgs::PoseStamped>("goal", 1);
	if (USE_RVIZ)
	{
		sub_goal_ = nhSub.subscribe("/move_base_simple/goal", 1, &PathPlanningClass::goal_callback, this);
	}
	else
	{
		goal_.pose.position.x = TARGET_POSITION_X;
		goal_.pose.position.y = TARGET_POSITION_Y;
		pub_goal_.publish(goal_);
	}

	sub_odom_ = nhSub.subscribe("position",1,&PathPlanningClass::__odom_callback,this);
	sub_cluster = nhSub.subscribe("classificationDataEstimateVelocity", 1, &PathPlanningClass::cluster_callback, this);
	sub_coefficient = nhSub.subscribe("/potential_coefficient", 1, &PathPlanningClass::coefficient_callback, this);
	sub_local_map_ = nhSub.subscribe("Localmap", 1, &PathPlanningClass::local_map_callback, this);
	sub_run_ = nhSub.subscribe("create_path", 1, &PathPlanningClass::__create_path_callback, this);
	sub_seg_ = nhSub.subscribe("segment", 1, &PathPlanningClass::__segment_callback, this);
	sub_state_ = nhSub.subscribe("state",1,&PathPlanningClass::__state_callback,this);
	
	//pub_odom= nhPub.advertise<nav_msgs::Odometry>("/potbot/odom", 1);
	pub_ShortestDistance = nhPub.advertise<geometry_msgs::Vector3>("ShortestDistance", 1);
	pub_PV = nhPub.advertise<potbot::PotentialValue>("PotentialValue", 1);
	pub_pf_ = nhPub.advertise<nav_msgs::GridCells>("pot", 1);
	pub_PP = nhPub.advertise<nav_msgs::Path>("Path", 1);

	if (path_planning_id == CSV_PATH)
	{
		std::string str_buf;
		std::string str_conma_buf;
		//std::cout<< PATH_PLANNING_FILE <<std::endl;
		std::ifstream ifs_csv_file(PATH_PLANNING_FILE);

		std_msgs::Header hd;
		hd.frame_id = "/map";
    	hd.stamp = ros::Time::now();
		while(hd.stamp.toSec() == 0)
		{
			hd.stamp = ros::Time::now();
		}
		robot_path_.header = hd;

		robot_path_.poses.resize(1000000);
		int cnt = 0;
		double line_buf[2];
		while (getline(ifs_csv_file, str_buf)) 
		{    
			
			std::istringstream i_stream(str_buf);// 「,」区切りごとにデータを読み込むためにistringstream型にする
			
			int i = 0;
			while (getline(i_stream, str_conma_buf, ',')) // 「,」区切りごとにデータを読み込む
			{
				//std::cout<< str_conma_buf <<std::endl;
				line_buf[i++] = std::stod(str_conma_buf);
			}
			robot_path_.poses[cnt].pose.position.x   = line_buf[0];
			robot_path_.poses[cnt++].pose.position.y = line_buf[1];
		}
		//std::cout<< cnt <<std::endl;
		robot_path_.poses.resize(cnt);
		
		//PP.data.resize(robot_path.size());
    	//PP.data = robot_path;
		publishPathPlan();
	}

	f_ = boost::bind(&PathPlanningClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

}
PathPlanningClass::~PathPlanningClass(){
}
