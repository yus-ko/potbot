// #include<potbot_localization/Localization.h>
// #include <potbot_lib/Utility.h>
#include <ros/ros.h>
// #include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>

std::string G_FRAME_ID_MAP			= "map",
			G_FRAME_ID_ODOM			= "odom",
			G_FRAME_ID_ROBOT_BASE	= "base_footprint",
			G_TOPIC_TWIST			= "cmd_vel",
			G_TOPIC_ODOM			= "odom";

ros::Publisher g_pub_odom;
nav_msgs::Odometry g_odom;

void get_RPY(geometry_msgs::Quaternion orientation, double &roll, double &pitch, double &yaw)
{
	tf2::Quaternion quat;
	tf2::convert(orientation, quat);
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

geometry_msgs::Quaternion get_Quat(double roll, double pitch, double yaw)
{
	tf2::Quaternion quat;
	quat.setRPY(roll, pitch, yaw);
	geometry_msgs::Quaternion orientation;
	tf2::convert(quat, orientation);
	return orientation;
}

double get_Yaw(geometry_msgs::Quaternion orientation)
{
	double roll, pitch, yaw;
	get_RPY(orientation, roll, pitch, yaw);
	return yaw;
}

void twist_callback(const geometry_msgs::Twist& msg)
{
	// ROS_INFO("twist callback");
	g_odom.header.frame_id = G_FRAME_ID_ODOM;
	g_odom.header.stamp = ros::Time::now();

	static double t_pre = g_odom.header.stamp.toSec();
	double dt = g_odom.header.stamp.toSec() - t_pre;

	g_odom.twist.twist = msg;

	double v = g_odom.twist.twist.linear.x;
	double omega = g_odom.twist.twist.angular.z;
	double theta = get_Yaw(g_odom.pose.pose.orientation) + omega*dt;
	double x = g_odom.pose.pose.position.x + v*cos(theta)*dt;
	double y = g_odom.pose.pose.position.y + v*sin(theta)*dt;

	g_odom.pose.pose.position.x = x;
	g_odom.pose.pose.position.y = y;
	g_odom.pose.pose.orientation = get_Quat(0,0,theta);

	g_pub_odom.publish(g_odom);

	t_pre = g_odom.header.stamp.toSec();

	geometry_msgs::TransformStamped tf_base_footprint;

    tf_base_footprint.header = g_odom.header;
    tf_base_footprint.child_frame_id = G_FRAME_ID_ROBOT_BASE;
    tf_base_footprint.transform.translation.x = g_odom.pose.pose.position.x;
    tf_base_footprint.transform.translation.y = g_odom.pose.pose.position.y;
    tf_base_footprint.transform.translation.z = g_odom.pose.pose.position.z;
    tf_base_footprint.transform.rotation = g_odom.pose.pose.orientation;
	static tf2_ros::TransformBroadcaster g_broadcaster;
    g_broadcaster.sendTransform(tf_base_footprint);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_twist_to_odometry");
    // LocalizationClass lc;
	// ros::spin();

	ros::NodeHandle n("~");
	
	double x=0,y=0,z=0,roll=0,pitch=0,yaw=0;
	n.getParam("FRAME_ID/MAP",			G_FRAME_ID_MAP);
	n.getParam("FRAME_ID/ODOM",			G_FRAME_ID_ODOM);
	n.getParam("FRAME_ID/ROBOT_BASE",	G_FRAME_ID_ROBOT_BASE);
	n.getParam("TOPIC/TWIST",			G_TOPIC_TWIST);
	n.getParam("TOPIC/ODOM",			G_TOPIC_ODOM);
	n.getParam("INITIAL_POSE/X",		x);
	n.getParam("INITIAL_POSE/Y",		y);
	n.getParam("INITIAL_POSE/Z",		z);
	n.getParam("INITIAL_POSE/ROLL",		roll);
	n.getParam("INITIAL_POSE/PITCH",	pitch);
	n.getParam("INITIAL_POSE/YAW",		yaw);

	ros::NodeHandle nh;
	ros::Subscriber sub_twist = nh.subscribe(G_TOPIC_TWIST,1,&twist_callback);
	g_pub_odom	 = nh.advertise<nav_msgs::Odometry>(G_TOPIC_ODOM, 1);

	nav_msgs::Odometry odom_init;
	odom_init.pose.pose.position.x = 0;
	odom_init.pose.pose.position.y = 0;
	odom_init.pose.pose.position.z = 0;
	odom_init.pose.pose.orientation = get_Quat(0,0,0);
	g_odom = odom_init;

	ros::spin();
	

	return 0;
}