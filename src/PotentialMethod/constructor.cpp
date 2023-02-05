#include<potbot/PotentialMethod.h>
#include <fstream>

PotentialMethodClass::PotentialMethodClass()
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
		path_planning_id = 0;
	}
	else if (PATH_PLANNING_METHOD == "potential_method")
	{
		path_planning_id = 1;
	}
	
	if (path_planning_id == 0)
	{
		std::string str_buf;
		std::string str_conma_buf;
		std::cout<< PATH_PLANNING_FILE <<std::endl;
		std::ifstream ifs_csv_file(PATH_PLANNING_FILE);

		robot_path.resize(1000000);
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
			robot_path[cnt].x   = line_buf[0];
			robot_path[cnt++].y = line_buf[1];
		}
		std::cout<< cnt <<std::endl;
		robot_path.resize(cnt);
		
		PP.data.resize(robot_path.size());
    	PP.data = robot_path;
	}

	if (ROBOT_NAME == "megarover")
	{
		robot_id = 0;
	}
	else if (ROBOT_NAME == "turtlebot3")
	{
		robot_id = 1;
	}

	sub_scan=nhSub.subscribe("/scan",1,&PotentialMethodClass::scan_callback,this);

	if (robot_id == 0)
	{
		if(IS_SIMULATOR)
		{
			if (!USE_AMCL) sub_encoder=nhSub.subscribe("/vmegarover/diff_drive_controller/odom",1,&PotentialMethodClass::encoder_callback_sim,this);
			if(PUBLISH_COMMAND) pub_cmd= nhPub.advertise<geometry_msgs::Twist>("/vmegarover/diff_drive_controller/cmd_vel", 1);
		}
		else
		{
			if (!USE_AMCL) sub_encoder=nhSub.subscribe("/rover_odo",1,&PotentialMethodClass::encoder_callback,this);
			if(PUBLISH_COMMAND) pub_cmd= nhPub.advertise<geometry_msgs::Twist>("/rover_twist", 1);
		}
	}
	else if (robot_id == 1)
	{
		if (!USE_AMCL) sub_encoder = nhSub.subscribe("/odom", 1, &PotentialMethodClass::encoder_callback_sim, this);
		if(PUBLISH_COMMAND) pub_cmd = nhPub.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}

	if (USE_AMCL) sub_encoder = nhSub.subscribe("/amcl_pose", 1, &PotentialMethodClass::pwcs_callback, this);

	sub_cluster = nhSub.subscribe("classificationDataEstimateVelocity", 1, &PotentialMethodClass::cluster_callback, this);
	sub_coefficient = nhSub.subscribe("/potential_coefficient", 1, &PotentialMethodClass::coefficient_callback, this);
	
	pub_odom= nhPub.advertise<nav_msgs::Odometry>("/autonomous_mobile_robot_2022/odom", 1);
	pub_ShortestDistance = nhPub.advertise<geometry_msgs::Vector3>("/autonomous_mobile_robot_2022/ShortestDistance", 1);
	pub_PV = nhPub.advertise<potbot::PotentialValue>("/autonomous_mobile_robot_2022/PotentialValue", 1);
	pub_PP = nhPub.advertise<potbot::PathPlan>("/autonomous_mobile_robot_2022/Path", 1);

	start_time = ros::WallTime::now();
	
}
PotentialMethodClass::~PotentialMethodClass(){
}
