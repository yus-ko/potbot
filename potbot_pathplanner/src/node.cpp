#include<potbot_pathplanner/PathPlanning.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_pp");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    PathPlanningClass rcc(buffer);
	ros::spin();

	return 0;
}