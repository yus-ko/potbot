#include<potbot/PathPlanning.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_pp");

    PathPlanningClass rcc;
	rcc.mainloop();

	return 0;
}