#include<autonomous_mobile_robot_2022/RobotControl.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_rc");

    RobotControlClass rcc;
    ros::spin();

	return 0;
}