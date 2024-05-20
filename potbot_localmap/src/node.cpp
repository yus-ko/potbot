#include<potbot_localmap/Localmap.h>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"potbot_lom");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    LocalmapClass lc(buffer);
	ros::spin();

	return 0;
}