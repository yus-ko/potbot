#include<autonomous_mobile_robot_2022/PointCloud.h>

//このプログラムは受け取ったデータを外部に送信します。
int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_pc");

    PointCloudClass pcc;
    ros::spin();

	return 0;
}