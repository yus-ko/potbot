#include<autonomous_mobile_robot_2022/RobotControl.h>

RobotControlClass::RobotControlClass()
{

	setLaunchParam();	// lanchファイルの読み込み

	if(IS_SIMULATOR)
	{
		sub_encoder=nhSub.subscribe("/vmegarover/diff_drive_controller/odom",1,&RobotControlClass::encoder_callback_sim,this);
		if(PUBLISH_COMMAND) pub_cmd= nhPub.advertise<geometry_msgs::Twist>("/vmegarover/diff_drive_controller/cmd_vel", 1);
	}
	else
	{
		sub_encoder=nhSub.subscribe("/rover_odo",1,&RobotControlClass::encoder_callback,this);
		if(PUBLISH_COMMAND) pub_cmd= nhPub.advertise<geometry_msgs::Twist>("/rover_twist", 1);
	}
	
	target_angle = atan(TARGET_POSITION_Y/(TARGET_POSITION_X + 10e-100));
	if (TARGET_POSITION_X < 0) 
	{
		MAX_VELOCITY = -MAX_VELOCITY;
		if (TARGET_POSITION_Y > 0)
		{
			target_angle += M_PI/2;
		}
		else if (TARGET_POSITION_Y < 0)
		{
			target_angle -= M_PI/2;
		}
	}
	pub_odom= nhPub.advertise<nav_msgs::Odometry>("/autonomous_mobile_robot_2022/odom", 1);

	start_time = ros::WallTime::now();
	
}
RobotControlClass::~RobotControlClass(){
}
