#include<autonomous_mobile_robot_2022/rviz.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_rviz");

    rvizClass rvc;
    ros::spin();

	return 0;
}