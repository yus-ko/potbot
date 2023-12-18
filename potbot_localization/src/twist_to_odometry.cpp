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
			G_FRAME_ID_ROBOT_BASE				= "base_link",
			G_TOPIC_SUB_TWIST					= "cmd_vel",
			G_TOPIC_PUB_ODOM					= "odom";

ros::Publisher g_pub_odom, g_pub_odom_rectangle, g_pub_odom_trapezoid, g_pub_imu;
nav_msgs::Odometry g_odom, g_odom_rectangle, g_odom_trapezoid;

void dead_reckoning_rectangle(geometry_msgs::TwistStamped vel_msg)
{
	g_odom_rectangle.header						= vel_msg.header;
	g_odom_rectangle.child_frame_id				= G_FRAME_ID_ROBOT_BASE;
	g_odom_rectangle.twist.twist				= vel_msg.twist;

	static double t_pre							= g_odom_rectangle.header.stamp.toSec();
	double dt									= g_odom_rectangle.header.stamp.toSec() - t_pre;

	double v									= g_odom_rectangle.twist.twist.linear.x;
	double omega								= g_odom_rectangle.twist.twist.angular.z;
	double theta								= potbot_lib::utility::get_Yaw(g_odom_rectangle.pose.pose.orientation) + omega*dt;
	double x									= g_odom_rectangle.pose.pose.position.x + v*cos(theta)*dt;
	double y									= g_odom_rectangle.pose.pose.position.y + v*sin(theta)*dt;

	g_odom_rectangle.pose.pose.position.x		= x;
	g_odom_rectangle.pose.pose.position.y		= y;
	g_odom_rectangle.pose.pose.orientation		= potbot_lib::utility::get_Quat(0,0,theta);

	g_pub_odom_rectangle.publish(g_odom_rectangle);

	t_pre										= g_odom_rectangle.header.stamp.toSec();

	// geometry_msgs::TransformStamped tf_base_footprint;
    // tf_base_footprint.header					= g_odom_rectangle.header;
    // tf_base_footprint.child_frame_id			= G_FRAME_ID_ROBOT_BASE;
    // tf_base_footprint.transform.translation.x	= g_odom_rectangle.pose.pose.position.x;
    // tf_base_footprint.transform.translation.y	= g_odom_rectangle.pose.pose.position.y;
    // tf_base_footprint.transform.translation.z	= g_odom_rectangle.pose.pose.position.z;
    // tf_base_footprint.transform.rotation		= g_odom_rectangle.pose.pose.orientation;
	// static tf2_ros::TransformBroadcaster g_broadcaster;
    // g_broadcaster.sendTransform(tf_base_footprint);
}

void dead_reckoning_trapezoid(geometry_msgs::TwistStamped vel_msg)
{
	g_odom_trapezoid.header						= vel_msg.header;
	g_odom_trapezoid.child_frame_id				= G_FRAME_ID_ROBOT_BASE;
	g_odom_trapezoid.twist.twist				= vel_msg.twist;

	static double t_pre							= g_odom_trapezoid.header.stamp.toSec();
	double dt									= g_odom_trapezoid.header.stamp.toSec() - t_pre;

	static double v_pre							= g_odom_trapezoid.twist.twist.linear.x;
	double v_now								= g_odom_trapezoid.twist.twist.linear.x;

	static double omega_pre						= g_odom_trapezoid.twist.twist.angular.z;
	double omega_now							= g_odom_trapezoid.twist.twist.angular.z;

	double theta								= potbot_lib::utility::get_Yaw(g_odom_trapezoid.pose.pose.orientation) + ((omega_now + omega_pre)*dt/2);
	double x									= g_odom_trapezoid.pose.pose.position.x + ((v_now + v_pre)*dt/2)*cos(theta);
	double y									= g_odom_trapezoid.pose.pose.position.y + ((v_now + v_pre)*dt/2)*sin(theta);

	g_odom_trapezoid.pose.pose.position.x		= x;
	g_odom_trapezoid.pose.pose.position.y		= y;
	g_odom_trapezoid.pose.pose.orientation		= potbot_lib::utility::get_Quat(0,0,theta);

	g_pub_odom_trapezoid.publish(g_odom_trapezoid);
	
	t_pre										= g_odom_trapezoid.header.stamp.toSec();
	v_pre										= v_now;
	omega_pre									= omega_now;
}

void twist_callback(const geometry_msgs::Twist& msg)
{
	// ROS_INFO("twist callback");
	geometry_msgs::TwistStamped tm;
	tm.header.frame_id							= G_FRAME_ID_ODOM;
	tm.header.stamp								= ros::Time::now();
	tm.twist									= msg;

	dead_reckoning_rectangle(tm);
	dead_reckoning_trapezoid(tm);
}

void odom_twist_callback(const nav_msgs::Odometry& msg)
{
	// ROS_INFO("odom twist callback");
	geometry_msgs::TwistStamped tm;
	tm.header									= msg.header;
	tm.twist									= msg.twist.twist;
	// tm.twist.linear.x *= 1.1;
	// tm.twist.angular.z *= 1.1;

	dead_reckoning_rectangle(tm);
	dead_reckoning_trapezoid(tm);
}

void imu_callback(const sensor_msgs::Imu& msg)
{
	// ROS_INFO("imu callback");
	sensor_msgs::Imu imu						= msg;
	imu.header.frame_id 						= "robot_5/imu_link";
	g_pub_imu.publish(imu);

}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_twist_to_odometry");
    // LocalizationClass lc;
	// ros::spin();

	ros::NodeHandle n("~");
	
	double x=0,y=0,z=0,roll=0,pitch=0,yaw=0;
	n.getParam("FRAME_ID/MAP",					G_FRAME_ID_MAP);
	n.getParam("FRAME_ID/ODOM",					G_FRAME_ID_ODOM);
	n.getParam("FRAME_ID/ROBOT_BASE",			G_FRAME_ID_ROBOT_BASE);
	n.getParam("TOPIC/SUB/TWIST",				G_TOPIC_SUB_TWIST);
	n.getParam("TOPIC/PUB/ODOM",				G_TOPIC_PUB_ODOM);
	n.getParam("INITIAL_POSE/X",				x);
	n.getParam("INITIAL_POSE/Y",				y);
	n.getParam("INITIAL_POSE/Z",				z);
	n.getParam("INITIAL_POSE/ROLL",				roll);
	n.getParam("INITIAL_POSE/PITCH",			pitch);
	n.getParam("INITIAL_POSE/YAW",				yaw);

	ros::NodeHandle nh;
	ros::Subscriber sub_twist					= nh.subscribe(G_TOPIC_SUB_TWIST,1,&twist_callback);
	ros::Subscriber sub_odom_twist				= nh.subscribe("/robot_5/odom",1,&odom_twist_callback);
	ros::Subscriber sub_imu						= nh.subscribe("/robot_5/mynteye/imu/data_raw",1,&imu_callback);
	g_pub_odom									= nh.advertise<nav_msgs::Odometry>(G_TOPIC_PUB_ODOM, 1);
	g_pub_odom_rectangle	 					= nh.advertise<nav_msgs::Odometry>("/gazebo_dead_reckoning/rectangle", 1);
	g_pub_odom_trapezoid	 					= nh.advertise<nav_msgs::Odometry>("/gazebo_dead_reckoning/trapezoid", 1);
	g_pub_imu	 								= nh.advertise<sensor_msgs::Imu>("/robot_5/imu", 1);

	nav_msgs::Odometry odom_init;
	odom_init.pose.pose							= potbot_lib::utility::get_Pose(0,0,0,0,0,0);
	g_odom										= odom_init;
	g_odom_rectangle							= odom_init;
	g_odom_trapezoid							= odom_init;

	ros::spin();
	

	return 0;
}