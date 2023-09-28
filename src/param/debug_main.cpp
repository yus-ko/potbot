#include <ros/ros.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_param");
	ros::NodeHandle n("~");

	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		loop_rate.sleep();
	}
    // PathPlanningClass rcc;
	// rcc.mainloop();

	return 0;
}