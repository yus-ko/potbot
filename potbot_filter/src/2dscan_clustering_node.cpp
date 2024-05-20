#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <potbot_filter/2dscan_clustering.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_clus");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    scan2dClass s2d(buffer);
	ros::spin();
	
	return 0;
}