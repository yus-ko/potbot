#include<potbot/PotentialMethod.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_pm");

    PotentialMethodClass rcc;
	ros::spin();

    // ros::Rate loop_rate(5);
	// while (ros::ok())
	// {
	// 	rcc.manage();
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }

	return 0;
}