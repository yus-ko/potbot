#include<potbot_filter/Filter.h>

FilterClass::FilterClass()
{
	
	setLaunchParam();	// lanchファイルの読み込み

    //tf2_ros::TransformListener tfListener(tf_buffer_);

	sub_odom_ = nhSub_.subscribe("position",1,&FilterClass::__odom_callback,this);
	sub_obstacle_ = nhSub_.subscribe("segment",1,&FilterClass::__obstacle_callback,this);
	sub_scan_=nhSub_.subscribe(TOPIC_SCAN,1,&FilterClass::__scan_callback,this);
	pub_state_ = nhPub_.advertise<potbot_msgs::StateArray>("state", 1);
	pub_scan0_ = nhPub_.advertise<sensor_msgs::LaserScan>("scan0", 1);
	pub_scan1_ = nhPub_.advertise<sensor_msgs::LaserScan>("scan1", 1);
	pub_segment_ = nhPub_.advertise<visualization_msgs::MarkerArray>("segment", 1);

	f_ = boost::bind(&FilterClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);
	
}
FilterClass::~FilterClass(){
}
