#include <autonomous_mobile_robot_2022/synchroImage.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_si");
	
    syncroImageClass sic; //syncro class

    ros::spin();

	return 0;
}