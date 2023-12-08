#include<potbot_localization/Localization.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_lo");
    LocalizationClass lc;
	nav_msgs::Odometry odom;
	double theta = potbot_lib::utility::get_Yaw(odom.pose.pose.orientation);
	ros::spin();

	return 0;
}