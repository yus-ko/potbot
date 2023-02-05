#include<autonomous_mobile_robot_2022/Localization.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_lo");

    LocalizationClass lc;
	ros::spin();

	return 0;
}