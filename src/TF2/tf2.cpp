#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void dynamic_broadcast(nav_msgs::Odometry odom)
{

	geometry_msgs::TransformStamped tf_map2robot;

    //std::string ns = ros::this_node::getNamespace();

	std_msgs::Header header_;
	header_ = odom.header;
	header_.frame_id = "/world_origin";

	double t = header_.stamp.toSec();
	double x = sin(t);
	double y = 0.75*sin(2*t);
	double z = 0;
	double yaw = 0.25*sin(4*t);

	tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    tf_map2robot.header = header_;
	tf_map2robot.child_frame_id = "/robot";
    tf_map2robot.transform.translation.x = x;
    tf_map2robot.transform.translation.y = y;
    tf_map2robot.transform.translation.z = z;
	tf_map2robot.transform.rotation.x = q.x();
    tf_map2robot.transform.rotation.y = q.y();
    tf_map2robot.transform.rotation.z = q.z();
    tf_map2robot.transform.rotation.w = q.w();

	static tf2_ros::TransformBroadcaster broadcaster_;
    broadcaster_.sendTransform(tf_map2robot);

}

void static_broadcast()
{
	std_msgs::Header header_;
	header_.frame_id = "/robot";
    header_.stamp = ros::Time::now();

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped tf_robot2lidar;
    tf_robot2lidar.header = header_;
    tf_robot2lidar.child_frame_id = "/lidar";
    tf_robot2lidar.transform.translation.x = 0.0;
    tf_robot2lidar.transform.translation.y = 0.0;
    tf_robot2lidar.transform.translation.z = 0.1;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    tf_robot2lidar.transform.rotation.x = q.x();
    tf_robot2lidar.transform.rotation.y = q.y();
    tf_robot2lidar.transform.rotation.z = q.z();
    tf_robot2lidar.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(tf_robot2lidar);

    geometry_msgs::TransformStamped tf_robot2stereo_camera;
    tf_robot2stereo_camera.header = header_;
    tf_robot2stereo_camera.child_frame_id = "/stereo_camera";
    tf_robot2stereo_camera.transform.translation.x = 0.1;
    tf_robot2stereo_camera.transform.translation.y = 0.0;
    tf_robot2stereo_camera.transform.translation.z = 0.3;
    q.setRPY(0, 0, 0);
    tf_robot2stereo_camera.transform.rotation.x = q.x();
    tf_robot2stereo_camera.transform.rotation.y = q.y();
    tf_robot2stereo_camera.transform.rotation.z = q.z();
    tf_robot2stereo_camera.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(tf_robot2stereo_camera);
}

void encoder_callback_sim(const nav_msgs::Odometry& msg)
{
	//ROS_INFO("%s",msg.header.frame_id.c_str());
    dynamic_broadcast(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf2_broadcast_node");
    ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/odom", 1, &encoder_callback_sim);


    ros::Rate rate(10.0);
	static_broadcast();
    while (nh.ok()) {
        rate.sleep();
		ros::spinOnce();
    }

    return 0;
}
