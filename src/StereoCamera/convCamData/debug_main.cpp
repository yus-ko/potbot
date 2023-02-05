#include <ros/ros.h>
#include <potbot/convCamData.h>
// #include "debug_method.cpp"
#include <iostream>

//このプログラムは受け取ったデータを外部に送信します。
int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_gr");

    convCamDataClass ccd;
    ros::spin();

	// reconfigure test のためコメントアウト
	// int lr_value = 20; //60 loop rate value
	// ros::param::param("loop_rate", lr_value, lr_value);
    // d_convCamDataClass ccd;
	// ROS_INFO_STREAM("convCamDataClass instance generated.\n");
	// ros::Rate loop_rate(lr_value);
	// // ros::Time ransac_time = ros::Time::now();
	// while (ros::ok()) {
	// 	ros::spinOnce();
	// 	if(ccd.subscribeSensorData()){
	// 		ROS_INFO_STREAM("RANSAC start");
	// 		ccd.groundEstimationRANSAC();
	// 		ccd.create2dMap();
	// 		ccd.publishConvCamData();
	// 		ccd.publishMaskImage();
	// 		ccd.publishGroundData();
	// 		ccd.d_smd2pcl();
	// 		ccd.d_publish_pcl();
	// 		ccd.d_mid2mat();
	// 		ccd.d_publish_img();
	// 	}
	// 	loop_rate.sleep();
	// }

	return 0;
}