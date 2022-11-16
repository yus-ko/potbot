#include<autonomous_mobile_robot_2022/PotentialMethod.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_pm");

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