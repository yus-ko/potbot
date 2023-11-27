#include<potbot_pcl/Clustering3D.h>

Clustering3DClass::Clustering3DClass()
{

	//ros::Duration(5).sleep();
	
	setLaunchParam();	// lanchファイルの読み込み

	sub_pcl2_	= nhSub_.subscribe(TOPIC_PCL2,1,&Clustering3DClass::__pcl2_callback,this);

	pub_marker_ = nhPub_.advertise<visualization_msgs::MarkerArray>("filtered_pcl", 1);
	pub_pcl_0_ = nhPub_.advertise<sensor_msgs::PointCloud2>("debug/pcl/DownSampling", 1);
	pub_pcl_1_ = nhPub_.advertise<sensor_msgs::PointCloud2>("debug/pcl/Plane_removal", 1);

	f_ = boost::bind(&Clustering3DClass::__param_callback, this, _1, _2);
	server_.setCallback(f_);

}
Clustering3DClass::~Clustering3DClass(){
}
