#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <potbot_filter/Filter.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_fi");

    potbot_filter::FilterClass fc;
	ros::spin();
	
	return 0;
}