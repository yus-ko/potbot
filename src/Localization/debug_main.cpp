#include<potbot/Localization.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_lo");

    LocalizationClass lc;
	ros::spin();

	return 0;
}