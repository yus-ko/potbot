#include<potbot_localmap/Localmap.h>

LocalmapClass::LocalmapClass()
{
	
	__get_param();	// lanchファイルの読み込み

	sub_obstacles_scan_	= nhSub_.subscribe("obstacle/scan/estimate",1,&LocalmapClass::__obstacles_scan_callback,this);
	sub_obstacles_pcl_	= nhSub_.subscribe("obstacle/pcl",1,&LocalmapClass::__obstacles_pcl_callback,this);

	pub_localmap_		= nhPub_.advertise<nav_msgs::OccupancyGrid>("Localmap", 1);

	f_ = boost::bind(&LocalmapClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);
	
}
LocalmapClass::~LocalmapClass(){
}
