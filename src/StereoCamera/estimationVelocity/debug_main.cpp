#include <ros/ros.h>
#include <potbot/velocityEstimation.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_estimate");
	
	// ROS_INFO("velocityEstimation define");
    velocityEstimation ec; //
    ros::spin();
	return 0;
}