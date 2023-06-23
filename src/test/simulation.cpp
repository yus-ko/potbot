#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
// #include <Eigen/Geometry>

Eigen::VectorXd u(double t)
{
	Eigen::VectorXd ans(2);
	ans << sin(t), cos(t);
	return ans;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_te");

	ros::NodeHandle nhPub;
	ros::Publisher pub_cmd = nhPub.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);

	ros::Rate loop_rate(50);
	
	ros::WallTime time_start = ros::WallTime::now();

	Eigen::VectorXd x(3);
	x << 0, 0, 0;
	Eigen::MatrixXd A(3,3);
	A << 1, 1, 1,
		 1, 1, 1,
		 1, 1, 1;

	ROS_INFO("開始");
	while (ros::ok())
	{
		geometry_msgs::Twist cmd;

		cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    	cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

		double time_now = ros::WallTime::now().toSec() - time_start.toSec();

		cmd.linear.x = 0.5*sin(2*M_PI*0.1*time_now);

		// if (time_now > 5 && time_now <= 10)
		// {
		// 	cmd.linear.x = 0.2;
		// 	//cmd.angular.z = M_PI_4;
		// }
		// else if (time_now > 15 && time_now <= 20)
		// {
		// 	cmd.linear.x = -0.2;
		// 	//cmd.angular.z = -M_PI_4;
		// }
		// else if (time_now > 25)
		// {
		// 	break;
		// }

		ROS_INFO("並進速度: %f", cmd.linear.x);
		ROS_INFO("回転角速度: %f", cmd.angular.z);
		pub_cmd.publish(cmd);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("終了");

	return 0;
}

