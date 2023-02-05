#include<potbot/PointCloud.h>

PointCloudClass::PointCloudClass()
{
	//subscriber
	sub=nhSub.subscribe("/mynteye/points/data_raw",1,&PointCloudClass::pcl_callback,this);
	sub_depthimage=nhSub.subscribe("/mynteye/depth/image_raw",1,&PointCloudClass::depthimage_callback,this);

	sub_odom=nhSub.subscribe("/autonomous_mobile_robot_2022/odom",1,&PointCloudClass::odom_callback,this);
	//sub_encoder=nhSub.subscribe("/encoder",1,&PointCloudClass::encoder_callback,this);
	//publisher
    pubpc= nhPub1.advertise<potbot::PointCloudData>("Extracted_PointCloud", 1);
	pubpc1= nhPub1.advertise<potbot::PointCloudData>("Created_PointCloud", 1);
	pub_clus= nhPub1.advertise<potbot::ClusterData>("Clustering_PointCloud", 1);
	// lanchファイルの読み込み
	setLaunchParam();


	// std::cout<<"Distance to the object is : " << DISTANCE_TO_OBJECT << "[m]" <<std::endl;
	// std::cout<<"Angle to the object is : " << ANGLE_TO_OBJECT << "[deg]" <<std::endl;
	// std::cout<<"Robot velocity is : " << ROBOT_VELOCITY << "[m/s]" <<std::endl;
	// std::cout<<"Height of the object is : " << OBJECTSIZE_HEIGHT << "[m]" <<std::endl;
	// std::cout<<"Widht of the object is : " << OBJECTSIZE_WIDTH << "[m]" <<std::endl;
	// std::cout<<"Depth o the object is : " << OBJECTSIZE_DEPTH << "[m]" <<std::endl;
	
}
PointCloudClass::~PointCloudClass(){
}
