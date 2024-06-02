#ifndef _H_CONTROLLER_
#define _H_CONTROLLER_

#include <potbot_lib/Utility.h>
#include <potbot_lib/DiffDriveController.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_msgs/ControllerConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <potbot_msgs/NavigationAction.h>

namespace potbot_controller
{
	typedef actionlib::SimpleActionServer<potbot_msgs::NavigationAction> NavServer;

	class Controller
	{

	private:
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_odom_, sub_path_, sub_goal_;

		ros::NodeHandle nhPub_;
		ros::Publisher pub_cmd_, pub_path_request_, pub_look_ahead_;

		tf2_ros::Buffer& tf_buffer_;

		NavServer* nav_server_;

		potbot_lib::Controller::DiffDriveController robot_controller_;

		geometry_msgs::Twist cmd_;

		nav_msgs::Path robot_path_;

		geometry_msgs::PoseStamped goal_;

		dynamic_reconfigure::Server<potbot_msgs::ControllerConfig> *dsrv_;

		nav_msgs::Odometry odom_;

		bool set_goal_ = false;

		// パラメーターサーバー参照
		bool publish_command_;
		double stop_margin_angle_, stop_margin_distance_, distance_to_lookahead_point_, distance_change_to_pose_alignment_;

		void __odom_callback(const nav_msgs::Odometry &msg);
		void __goal_callback(const geometry_msgs::PoseStamped &msg);
		void __path_callback(const nav_msgs::Path &msg);
		void __param_callback(const potbot_msgs::ControllerConfig &param, uint32_t level);
		void __publish_path_request();
		void __publishcmd();

		void __execute_callback(const potbot_msgs::NavigationGoalConstPtr& current_goal);

		void __LineFollowing();
		void __PoseAlignment();

	public:
		Controller(tf2_ros::Buffer& tf, const std::string& name = "");
		~Controller();

		void manage();
		void controller();
	};
};

#endif // _H_CONTROLLER_
