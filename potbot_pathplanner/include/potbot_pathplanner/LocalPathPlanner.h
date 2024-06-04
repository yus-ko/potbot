#ifndef _H_PATHPLANNING_
#define _H_PATHPLANNING_

#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <potbot_lib/PathPlanner.h>
#include <potbot_msgs/StateArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_pathplanner/PathPlanningConfig.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

namespace potbot_pathplanner
{
    class LocalPathPlanner{

        private:
            tf2_ros::Buffer& tf_buffer_;
            
            ros::NodeHandle nhSub;
            ros::Subscriber sub_goal_, sub_local_map_, sub_seg_, sub_state_, sub_run_;

            ros::NodeHandle nhPub;
            ros::Publisher pub_cmd_, pub_path_, pub_potential_, pub_attraction_field_, pub_repulsion_field_, pub_potential_field_, pub_path_raw_;

            potbot_lib::PathPlanner::APFPathPlanner* apf_;

            geometry_msgs::PoseStamped goal_;

            nav_msgs::OccupancyGrid local_map_;

            double max_path_length_ = 6.0;
            double potential_field_resolution_ = 0.05;
            size_t potential_field_rows_ = 240;
            size_t potential_field_cols_ = 240;
            size_t path_search_range_ = 1;
            size_t collision_count_to_replanning_ = 10;
            double hit_distance_to_replanning_ = 0.1;
            bool sync_create_path_ = false;
            bool sync_create_apf_ = false;
            double path_plan_cycle_time_ = 0.5;

            size_t hit_count_ = 0;

            nav_msgs::Path robot_path_world_coord_, robot_path_;

            visualization_msgs::MarkerArray obstacles_;

            double rho_zero_=0.3, eta_=0.02, kp_=0.1;
            nav_msgs::GridCells potential_field_;
            std::vector<std::vector<bool>> potential_field_info_;

            int max_path_index_ = 50;
            double wu_=1, w_theta_=0;

            potbot_msgs::StateArray obstacle_state_;

            dynamic_reconfigure::Server<potbot_pathplanner::PathPlanningConfig> *dsrv_;

            std::string frame_id_global_ = "map", frame_id_robot_base_ = "base_link";
            
            void __param_callback(const potbot_pathplanner::PathPlanningConfig& param, uint32_t level);
            void __segment_callback(const visualization_msgs::MarkerArray& msg);
            void __state_callback(const potbot_msgs::StateArray& msg);
            void __create_path_callback(const std_msgs::Empty& msg);
            void __goal_callback(const geometry_msgs::PoseStamped& msg);
            void __local_map_callback(const nav_msgs::OccupancyGrid& msg);

            std::vector<nav_msgs::Odometry> __get_ObstacleList();
            int __create_PotentialField();
            void __create_Path();
            bool __PathCollision();

        public:
            LocalPathPlanner(tf2_ros::Buffer& tf, const std::string& name = "");
            ~LocalPathPlanner();
    };
};

#endif // _H_PATHPLANNING_
