#include<potbot/Controller.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_co");

    ControllerClass cc;
	ros::spin();

	return 0;
}