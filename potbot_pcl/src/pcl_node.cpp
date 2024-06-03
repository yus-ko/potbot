#include<potbot_pcl/Clustering3D.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_cl3d");

    Clustering3DClass cl3d;
	ros::spin();

	return 0;
}