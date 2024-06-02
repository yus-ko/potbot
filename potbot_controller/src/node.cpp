#include<potbot_controller/Controller.h>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"potbot_co");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    potbot_controller::Controller cc(buffer);
	ros::spin();

	return 0;
}