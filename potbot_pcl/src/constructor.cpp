#include<potbot_pcl/Clustering3D.h>

Clustering3DClass::Clustering3DClass()
{

	//ros::Duration(5).sleep();
	
	setLaunchParam();	// lanchファイルの読み込み

	sub_pcl2_	= nhSub.subscribe(TOPIC_PCL2,1,&Clustering3DClass::__pcl2_callback,this);

	pub_marker_ = nhPub.advertise<visualization_msgs::MarkerArray>("filtered_pcl", 1);
	pub_marker_0_ = nhPub.advertise<sensor_msgs::PointCloud2>("debug_pcl", 1);

	f_ = boost::bind(&Clustering3DClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

}
Clustering3DClass::~Clustering3DClass(){
}
