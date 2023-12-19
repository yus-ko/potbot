#include<potbot_controller/Controller.h>
#include <fstream>

ControllerClass::ControllerClass()
{
	
	__get_param();	// lanchファイルの読み込み

	sub_odom_			= nhSub_.subscribe(TOPIC_ODOM,						1,&ControllerClass::__odom_callback,this);
	sub_path_			= nhSub_.subscribe("Path",							1,&ControllerClass::__path_callback,this);
	sub_goal_			= nhSub_.subscribe("move_base_simple/goal",			1,&ControllerClass::__goal_callback, this);

	pub_path_request_	= nhPub_.advertise<std_msgs::Empty>(				"create_path", 1);
	pub_cmd_			= nhPub_.advertise<geometry_msgs::Twist>(			TOPIC_CMD_VEL, 1);
	pub_look_ahead_		= nhPub_.advertise<visualization_msgs::Marker>(		"LookAhead", 1);
	//cmd_topic_name = "/vmegarover/diff_drive_controller/cmd_vel";
	//cmd_topic_name = "/rover_twist";
	//cmd_topic_name = "/beego/cmd_vel";

	f_ = boost::bind(&ControllerClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

	static tf2_ros::TransformListener tfListener(tf_buffer_);
	try
	{
		tf_buffer_.lookupTransform(FRAME_ID_GLOBAL, FRAME_ID_ROBOT_BASE, ros::Time(0), ros::Duration(60));
	}
	catch (tf2::TransformException &ex) 
	{
		ROS_WARN("tf unavailable: %s", ex.what());
  	}

}
ControllerClass::~ControllerClass(){
}
