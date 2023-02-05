#include<potbot/PointCloud.h>

//このプログラムは受け取ったデータを外部に送信します。
int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_pc");

    PointCloudClass pcc;
    ros::spin();

	return 0;
}