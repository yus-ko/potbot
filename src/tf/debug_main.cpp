#include<autonomous_mobile_robot_2022/tf.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_2022_tf");

    TFClass tfc;
	//ros::spin();

    ros::Rate loop_rate(5);
	while (ros::ok())
	{
		tfc.manage();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}