#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <potbot_lib/PathPlanner.h>
#include <potbot_msgs/StateArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_msgs/PathPlanningConfig.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

class PathPlanningClass{

    private:
        tf2_ros::Buffer tf_buffer_;
        
		ros::NodeHandle nhSub;
		ros::Subscriber sub_goal_, sub_local_map_, sub_odom_, sub_seg_, sub_state_, sub_run_;

		ros::NodeHandle nhPub;
        ros::Publisher pub_cmd_, pub_path_, pub_potential_, pub_attraction_field_, pub_repulsion_field_, pub_potential_field_;

        std_msgs::Header header_;

        geometry_msgs::PoseStamped goal_;

        nav_msgs::OccupancyGrid local_map_;

        nav_msgs::Odometry odom_;

        int path_planning_id_ = potbot_lib::POTENTIAL_METHOD;

        double max_path_length_ = 6.0;
        double potential_field_resolution_ = 0.05;
        size_t potential_field_rows_ = 240;
        size_t potential_field_cols_ = 240;
        size_t path_search_range_ = 1;
        size_t collision_count_to_replanning_ = 10;
        double hit_distance_to_replanning_ = 0.1;

        size_t hit_count_ = 0;

        nav_msgs::Path robot_path_world_coord_, robot_path_;

        visualization_msgs::MarkerArray obstacles_;

        double rho_zero_=0.3, eta_=0.02, kp_=0.1;
        nav_msgs::GridCells potential_field_;
        std::vector<std::vector<bool>> potential_field_info_;

        int max_path_index_ = 50;
        double wu_=1, w_theta_=0;

        potbot_msgs::StateArray obstacle_state_;

        dynamic_reconfigure::Server<potbot_msgs::PathPlanningConfig> server_;
  	    dynamic_reconfigure::Server<potbot_msgs::PathPlanningConfig>::CallbackType f_;

        std::string PATH_PLANNING_METHOD, PATH_PLANNING_FILE, FRAME_ID_GLOBAL, FRAME_ID_ROBOT_BASE, TOPIC_ODOM, TOPIC_GOAL;
        double TARGET_POSITION_X, TARGET_POSITION_Y, TARGET_POSITION_YAW;
        
        void __odom_callback(const nav_msgs::Odometry& msg);
        void __param_callback(const potbot_msgs::PathPlanningConfig& param, uint32_t level);
        void __segment_callback(const visualization_msgs::MarkerArray& msg);
        void __state_callback(const potbot_msgs::StateArray& msg);
        void __create_path_callback(const std_msgs::Empty& msg);

        double __nCr(double n, double r);
        void __bezier(nav_msgs::Path& points);

        std::vector<nav_msgs::Odometry> __get_ObstacleList(int mode);
        double __get_ShortestDistanceToObstacle(double x, double y, std::vector<geometry_msgs::Vector3> &obstacles);
        int __get_PotentialFiledIndex(double x, double y);
        int __create_PotentialField();
        void __create_Path();
        void __create_Path_used_weight();
        bool __PathCollision();

        void run();

    public:
        PathPlanningClass();
        ~PathPlanningClass();

        void goal_callback(const geometry_msgs::PoseStamped& msg);
        void local_map_callback(const nav_msgs::OccupancyGrid& msg);

        void publishPathPlan();
};
