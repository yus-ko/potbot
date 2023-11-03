#include <potbot_lib/Utility.h>
#include <potbot_msgs/beego_encoder.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_localization/LocalizationConfig.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_map_link");
    // LocalizationClass lc;
	// ros::spin();

	ros::NodeHandle n("~");
	std::string FRAME_ID_MAP = "map", FRAME_ID_ODOM = "odom";
	double x=0,y=0,z=0,roll=0,pitch=0,yaw=0;
	n.getParam("FRAME_ID/MAP",FRAME_ID_MAP);
	n.getParam("FRAME_ID/ODOM",FRAME_ID_ODOM);
	n.getParam("INITIAL_POSE/X",x);
	n.getParam("INITIAL_POSE/Y",y);
	n.getParam("INITIAL_POSE/Z",z);
	n.getParam("INITIAL_POSE/ROLL",roll);
	n.getParam("INITIAL_POSE/PITCH",pitch);
	n.getParam("INITIAL_POSE/YAW",yaw);
	while(ros::ok())
	{
		static tf2_ros::StaticTransformBroadcaster static_broadcaster;

		geometry_msgs::TransformStamped tf_map2odom;
		tf_map2odom.header.stamp = ros::Time::now();
		tf_map2odom.header.frame_id = FRAME_ID_MAP;
		tf_map2odom.child_frame_id = FRAME_ID_ODOM;
		tf_map2odom.transform.translation.x = x;
		tf_map2odom.transform.translation.y = y;
		tf_map2odom.transform.translation.z = z;
		tf2::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		tf_map2odom.transform.rotation.x = q.x();
		tf_map2odom.transform.rotation.y = q.y();
		tf_map2odom.transform.rotation.z = q.z();
		tf_map2odom.transform.rotation.w = q.w();
		static_broadcaster.sendTransform(tf_map2odom);
		
	}
	

	return 0;
}