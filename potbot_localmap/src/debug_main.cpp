#include<potbot_localmap/Localmap.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_lom");
    LocalmapClass lc;
	ros::spin();

	return 0;
}