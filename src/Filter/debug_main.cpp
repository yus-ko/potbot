#include<potbot/Filter.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_fi");

    FilterClass fc;

	fc.mainloop();
	
	return 0;
}