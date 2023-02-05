#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <potbot/test.h>

//ros::NodeHandle nhPub, nhSub;

//std::numeric_limits<double>::quiet_NaN()
test::test()
{
	pub_odom = nhPub.advertise<nav_msgs::Odometry>("/potbot/odom", 1);
	pub_cmd = nhPub.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	sub_encoder = nhSub.subscribe("/odom",1,&test::encoder_callback,this);
	start_time = ros::WallTime::now();

}
test::~test(){
}

void test::encoder_callback(const nav_msgs::Odometry& msg)
{

	nav_msgs::Odometry encoder_value = msg;

	ros::Time now = ros::Time::now();
    odom.header.stamp = now;
    odom.header.frame_id = "/odom";

	if (encoder_first)
	{
		odom.twist.twist.linear.x = odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = odom.twist.twist.angular.y = odom.twist.twist.angular.z = 0.0;
		odom.pose.pose.position.x = odom.pose.pose.position.y = odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation.x = odom.pose.pose.orientation.y = odom.pose.pose.orientation.z = 0.0;
		odom.pose.pose.orientation.w = 1.0;
	}
	else
	{
		double encoder_deltatime = now.toSec() - encoder_time_pre.toSec();

		odom.pose.pose.orientation.z += (bottom_omega + encoder_value.twist.twist.angular.z) * encoder_deltatime / 2;
		double vel = (bottom_v + encoder_value.twist.twist.linear.x) * encoder_deltatime / 2;
		odom.pose.pose.position.x += vel * cos(odom.pose.pose.orientation.z);
		odom.pose.pose.position.y += vel * sin(odom.pose.pose.orientation.z);
	
	}

	bottom_v = encoder_value.twist.twist.linear.x;
	bottom_omega = encoder_value.twist.twist.angular.z;
	ros::Time encoder_time_pre = ros::Time::now();

	encoder_time_pre = now;
	encoder_first = false;
	pub_odom.publish(odom);




	cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
	cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

	double time_now = ros::WallTime::now().toSec() - start_time.toSec();
	if (time_now > 1 && time_now <= 6)
	{
		cmd.angular.z = 2*M_PI/5;
	}
	else if (time_now > 7 && time_now <= 12)
	{
		cmd.linear.x = 0.2;
	}

	ROS_INFO("並進速度: %f", cmd.linear.x);
	ROS_INFO("回転角速度: %f", cmd.angular.z);
	pub_cmd.publish(cmd);
	

}

int main(int argc,char **argv){

	ros::init(argc,argv,"potbot_te");

    test te;
	ros::spin();

	ROS_INFO("開始");
	ROS_INFO("終了");

	return 0;
}

