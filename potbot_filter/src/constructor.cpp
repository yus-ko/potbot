#include<potbot_filter/Filter.h>

FilterClass::FilterClass()
{
	
	__get_param();	// lanchファイルの読み込み

	sub_obstacle_				= nhSub_.subscribe("obstacle/scan/clustering",			1,&FilterClass::__obstacle_callback,this);
	
	pub_state_					= nhPub_.advertise<potbot_msgs::StateArray>(			"state", 1);
	pub_state_markers_			= nhPub_.advertise<visualization_msgs::MarkerArray>(	"state/marker", 1);
	pub_obstacles_pcl_			= nhPub_.advertise<potbot_msgs::ObstacleArray>(			"obstacle/pcl/estimate", 1);
	pub_obstacles_scan_			= nhPub_.advertise<potbot_msgs::ObstacleArray>(			"obstacle/scan/estimate", 1);
}
FilterClass::~FilterClass(){
}
