#include<potbot/Controller.h>
#include <fstream>

ControllerClass::ControllerClass()
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

	sub_odom_=nhSub_.subscribe("/potbot/odom",1,&ControllerClass::odom_callback,this);

	if (robot_id_ == 0)
	{
		if(IS_SIMULATOR)
		{
			//sub_encoder_=nhSub_.subscribe("/vmegarover/diff_drive_controller/odom",1,&ControllerClass::encoder_callback_sim,this);
		}
		else
		{
			//sub_encoder_=nhSub_.subscribe("/rover_odo",1,&ControllerClass::encoder_callback,this);
		}
	}
	else if (robot_id_ == 1)
	{
		//sub_encoder_ = nhSub_.subscribe("/odom", 1, &ControllerClass::encoder_callback_sim, this);
	}

	pub_cmd_= nhPub_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	// double x[] = {1,4,7,10,13};
	// double y[] = {1,3,-3,5,-5};
	// int size = sizeof(x)/sizeof(x[0]);
	// robot_path_.resize(size);
	// for (int i = 0; i < size; i++)
	// {
	// 	robot_path_[i].x = x[i];
	// 	robot_path_[i].y = y[i];
	// }

	std::string str_buf;
	std::string str_conma_buf;
	std::cout<< PATH_PLANNING_FILE <<std::endl;
	std::ifstream ifs_csv_file(PATH_PLANNING_FILE);

	robot_path_.resize(1000000);
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
		robot_path_[cnt].x   = line_buf[0];
		robot_path_[cnt++].y = line_buf[1];
	}
	std::cout<< cnt <<std::endl;
	robot_path_.resize(cnt);

}
ControllerClass::~ControllerClass(){
}
