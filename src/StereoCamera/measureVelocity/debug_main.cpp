#include <ros/ros.h>
#include <potbot/measurementVelocity.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_mv");
	//
    measurementVelocity mv; //
    ros::spin();
	// while(ros::ok())
	// {
	// 	ROS_INFO("subscribeClusterData");
	// 	mv.subscribeClusterData();
	// 	// ROS_INFO("subscribeImageMatchingData");
	// 	// mv.subscribeImageMatchingData();
	// 	ROS_INFO("isCurClstr");
	// 	if(!mv.isCurClstr()){
	// 		continue;
	// 	}
	// 	ROS_INFO("creatClstrMap");
	// 	mv.creatClstrMap();
	// 	ROS_INFO("mv.isPrvClstr() && mv.isMatchData()");
	// 	if(mv.isPrvClstr()){// && mv.isMatchData()){
	// 		ROS_INFO("matchingClstr");
	// 		mv.matchingClstr();
	// 		ROS_INFO("measurementProcess");
	// 		mv.measurementProcess();
	// 		ROS_INFO("visualizedVelocities");
	// 		mv.visualizedVelocities();
	// 	}
	// 	ROS_INFO("renewClstrData");
	// 	mv.renewClstrData();
	// }
	return 0;
}