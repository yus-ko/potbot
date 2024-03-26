// #include<potbot_localization/Localization.h>
#include <potbot_lib/Utility.h>
#include <ros/ros.h>
// #include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

std::string G_FRAME_ID_MAP						= "map",
			G_FRAME_ID_ODOM						= "odom",
			G_FRAME_ID_BASE_FOOTPRINT			= "base_footprint",
			G_FRAME_ID_ROBOT_BASE				= "base_link",
			G_TOPIC_SUB_TWIST					= "cmd_vel",
			G_TOPIC_SUB_INITPOSE				= "initialpose",
			G_TOPIC_PUB_ODOM					= "odom",
			G_DEAD_RECKONING					= "rectangle";

ros::Publisher g_pub_odom;
nav_msgs::Odometry g_odom;

void dead_reckoning(geometry_msgs::TwistStamped vel_msg)
{
	g_odom.header								= vel_msg.header;
	g_odom.child_frame_id						= G_FRAME_ID_BASE_FOOTPRINT;
	g_odom.twist.twist							= vel_msg.twist;

	static double t_pre							= g_odom.header.stamp.toSec();
	double dt									= g_odom.header.stamp.toSec() - t_pre;

	double v									= g_odom.twist.twist.linear.x;
	double omega								= g_odom.twist.twist.angular.z;

	static double v_pre							= v;
	static double omega_pre						= omega;

	double theta,x,y;
	if (G_DEAD_RECKONING == "rectangle")
	{
		theta									= potbot_lib::utility::get_Yaw(g_odom.pose.pose.orientation) + omega*dt;
		x										= g_odom.pose.pose.position.x + v*cos(theta)*dt;
		y										= g_odom.pose.pose.position.y + v*sin(theta)*dt;
	}
	else if (G_DEAD_RECKONING == "trapezoid")
	{
		theta									= potbot_lib::utility::get_Yaw(g_odom.pose.pose.orientation) + ((omega + omega_pre)*dt/2);
		x										= g_odom.pose.pose.position.x + ((v + v_pre)*dt/2)*cos(theta);
		y										= g_odom.pose.pose.position.y + ((v + v_pre)*dt/2)*sin(theta);
	}
	


	

	g_odom.pose.pose.position.x					= x;
	g_odom.pose.pose.position.y					= y;
	g_odom.pose.pose.orientation				= potbot_lib::utility::get_Quat(0,0,theta);

	t_pre										= g_odom.header.stamp.toSec();
	v_pre										= v;
	omega_pre									= omega;

}

void twist_callback(const geometry_msgs::Twist& msg)
{
	// ROS_INFO("twist callback");
	geometry_msgs::TwistStamped tm;
	tm.header.frame_id							= G_FRAME_ID_ODOM;
	tm.header.stamp								= ros::Time::now();
	tm.twist									= msg;

	dead_reckoning(tm);

	g_pub_odom.publish(g_odom);
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	// ROS_INFO("imu callback");
	g_odom.pose = msg.pose;

}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_twist_to_odometry");
    // LocalizationClass lc;
	// ros::spin();

	ros::NodeHandle n("~");
	
	double x=0,y=0,z=0,roll=0,pitch=0,yaw=0;
	n.getParam("FRAME_ID/MAP",					G_FRAME_ID_MAP);
	n.getParam("FRAME_ID/ODOM",					G_FRAME_ID_ODOM);
	n.getParam("FRAME_ID/BASE_FOOTPRINT",		G_FRAME_ID_BASE_FOOTPRINT);
	n.getParam("FRAME_ID/ROBOT_BASE",			G_FRAME_ID_ROBOT_BASE);
	n.getParam("TOPIC/SUB/TWIST",				G_TOPIC_SUB_TWIST);
	n.getParam("TOPIC/SUB/INITIAL_POSE",		G_TOPIC_SUB_INITPOSE);
	n.getParam("TOPIC/PUB/ODOM",				G_TOPIC_PUB_ODOM);
	n.getParam("INITIAL/POSE/X",				x);
	n.getParam("INITIAL/POSE/Y",				y);
	n.getParam("INITIAL/POSE/Z",				z);
	n.getParam("INITIAL/POSE/ROLL",				roll);
	n.getParam("INITIAL/POSE/PITCH",			pitch);
	n.getParam("INITIAL/POSE/YAW",				yaw);

	ros::NodeHandle nh;
	ros::Subscriber sub_twist					= nh.subscribe(G_TOPIC_SUB_TWIST,1,&twist_callback);
	ros::Subscriber sub_pose					= nh.subscribe(G_TOPIC_SUB_INITPOSE,1,&pose_callback);
	g_pub_odom									= nh.advertise<nav_msgs::Odometry>(G_TOPIC_PUB_ODOM, 1);

	g_odom.pose.pose							= potbot_lib::utility::get_Pose(x,y,z,roll,pitch,yaw);

	ros::spin();
	
	return 0;
}