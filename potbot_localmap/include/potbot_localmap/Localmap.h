#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <potbot_msgs/ObstacleArray.h>
#include <potbot_msgs/LocalmapConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>


class LocalmapClass{

    private:
        
        tf2_ros::Buffer tf_buffer_;

		ros::NodeHandle nhSub_;
		ros::Subscriber sub_obstacles_scan_, sub_obstacles_pcl_;
        
        ros::NodeHandle nhPub_;
		ros::Publisher pub_localmap_;

        tf2_ros::TransformBroadcaster broadcaster_;

        potbot_msgs::ObstacleArray obstacles_scan_, obstacles_pcl_;

        dynamic_reconfigure::Server<potbot_msgs::LocalmapConfig> server_;
  	    dynamic_reconfigure::Server<potbot_msgs::LocalmapConfig>::CallbackType f_;

        double apply_cluster_to_localmap_ = 1.5;
        double prediction_time_ = 2.0;
        double max_estimated_linear_velocity_ = 1.0;
        double max_estimated_angular_velocity_ = 2.0;

        std::string frame_id_robot_base_ = "base_link";

        void __obstacles_scan_callback(const potbot_msgs::ObstacleArray& msg);
        void __obstacles_pcl_callback(const potbot_msgs::ObstacleArray& msg);

        void __param_callback(const potbot_msgs::LocalmapConfig& param, uint32_t level);

    public:
        LocalmapClass();
        ~LocalmapClass(){};

        
};
