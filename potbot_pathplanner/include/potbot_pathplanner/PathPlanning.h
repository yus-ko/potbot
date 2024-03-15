//include haeders
#include <ros/ros.h>
#include <ros/package.h>
#include <potbot_lib/Utility.h>
#include <potbot_lib/PathPlanner.h>
#include <potbot_msgs/StateArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dynamic_reconfigure/server.h>
#include <potbot_pathplanner/PathPlanningConfig.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sensor_msgs/PointCloud2.h>

//クラスの定義
class PathPlanningClass{

    private:

        tf::TransformListener tflistener;
        tf2_ros::Buffer tf_buffer_;
        
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub_encoder, sub_scan_, sub_coefficient, sub_goal_, sub_local_map_, sub_odom_, sub_seg_, sub_state_, sub_run_;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub_cmd, pub_odom, pub_PP, pub_cmd_, pub_potential_, pub_attraction_field_, pub_repulsion_field_, pub_potential_field_;

        std_msgs::Header header_;

        geometry_msgs::PoseStamped goal_;

        nav_msgs::OccupancyGrid local_map_;

        ros::Time encoder_time_pre, potential_time_pre, manage_time, manage_time_pre;
        geometry_msgs::Twist encoder_value, cmd;
        nav_msgs::Odometry odom, odom_pre, odom_msg, odom_;
        double bottom_v = 0,bottom_omega = 0;
        bool odometry_firsttime = true;
        sensor_msgs::LaserScan scan, scan_msg;
        std::vector<std::vector<float>> obstacle;
        geometry_msgs::Vector3 U_pre, ShortestDistance;

        geometry_msgs::PoseWithCovarianceStamped pwcs_msg;

        int path_planning_id_ = potbot_lib::POTENTIAL_METHOD;

        bool encoder_first = false, scan_first = false;

        double error_pos_x_pre = 0.0, error_pos_y_pre = 0.0, error_vel_pre = 0.0, error_angular_pre = 0.0, integral_vel_error = 0.0, integral_angular_error = 0.0;
        double encoder_deltatime;

        bool zero_potential = true;
        double f_time_pre = -999999999999999;

        double cont_pos_time_pre = -999999999999999, pos_x_pre = 0, pos_y_pre = 0, vel_x_pre = 0, vel_y_pre = 0;

        double start_x,start_y,start_theta,target_distance,target_angle, sub_goal_x , sub_goal_y;
        int done_cont_pos_cnt = 0;
        bool done_cont_pos = true;

        double integral_theta_error = 0;

        double distance_to_obstacle;
        int obstacle_index = 0, obstacle_size = 1000;

        double max_path_length_ = 6.0;
        double potential_field_resolution_ = 0.05;
        size_t potential_field_rows_ = 240;
        size_t potential_field_cols_ = 240;
        size_t path_search_range_ = 1;
        size_t collision_count_to_replanning_ = 10;
        double hit_distance_to_replanning_ = 0.1;

        size_t hit_count_ = 0;

        std::vector<int> exploration_arr;
        //std::vector<geometry_msgs::Vector3> robot_path;
        nav_msgs::Path robot_path_world_coord_, robot_path_;
        std::vector<int> centered;
        geometry_msgs::Vector3 sub_start;
        int robot_path_index = 0, robot_path_index_pre = 0;
        
        ros::Time path_update_timestamp;

        std::vector<double> scan_range_maximum_angle;
        int scan_range_maximum_angle_size = 10, scan_range_maximum_angle_index = 0;
        bool moving_average = false, wall_exists = false;

        visualization_msgs::MarkerArray obstacles_;

        double rho_zero_=0.3, eta_=0.02, kp_=0.1;
        nav_msgs::GridCells potential_field_;
        std::vector<std::vector<bool>> potential_field_info_;

        int max_path_index_ = 50;
        double wu_=1, w_theta_=0;

        double test_vx_=0,test_vy_=0,test_theta_=0,a_=1,b_=1,c_=1;

        bool sync_createpath_ = false;

        potbot_msgs::StateArray obstacle_state_;

        dynamic_reconfigure::Server<potbot_pathplanner::PathPlanningConfig> server_;
  	    dynamic_reconfigure::Server<potbot_pathplanner::PathPlanningConfig>::CallbackType f_;

        std::string PATH_PLANNING_METHOD, PATH_PLANNING_FILE, FRAME_ID_GLOBAL, FRAME_ID_ROBOT_BASE, TOPIC_SCAN, TOPIC_ODOM, TOPIC_GOAL;
        bool USE_AMCL, USE_RVIZ;
        double TARGET_POSITION_X, TARGET_POSITION_Y, TARGET_POSITION_YAW;
        double POTENTIAL_FIELD_WIDTH, POTENTIAL_FIELD_DIVDE_X, POTENTIAL_FIELD_DIVDE_Y;
        
        void __odom_callback(const nav_msgs::Odometry& msg);
        void __scan_callback(const sensor_msgs::LaserScan& msg);
        void __param_callback(const potbot_pathplanner::PathPlanningConfig& param, uint32_t level);
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
        void __create_PathFromCSV();
        bool __PathCollision();
        void __set_Param();

        void run();
        
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        PathPlanningClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~PathPlanningClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        void goal_callback(const geometry_msgs::PoseStamped& msg);
        void local_map_callback(const nav_msgs::OccupancyGrid& msg);
        
        void mainloop();
        void line_following();

        void publishPathPlan();
};
