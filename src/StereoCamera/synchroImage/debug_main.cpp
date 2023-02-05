#include <potbot/synchroImage.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_si");
	
    syncroImageClass sic; //syncro class

    ros::spin();

	return 0;
}