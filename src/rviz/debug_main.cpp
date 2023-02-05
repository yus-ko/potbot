#include<potbot/rviz.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_rviz");

    rvizClass rvc;
    ros::spin();

	return 0;
}