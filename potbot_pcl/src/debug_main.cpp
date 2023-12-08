#include<potbot_pcl/Clustering3D.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_cl3d");

    Clustering3DClass cl3d;
	cl3d.mainloop();

	return 0;
}