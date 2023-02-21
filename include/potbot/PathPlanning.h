//include haeders
#include <potbot/Utility.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <potbot/PotentialValue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <potbot/ClassificationVelocityData.h>

//クラスの定義
class PathPlanningClass{

    private:
        tf::TransformBroadcaster robotState_broadcaster;
        tf::TransformBroadcaster LRF_broadcaster;
        tf::TransformBroadcaster StereoCamera_broadcaster;
        tf::TransformBroadcaster obstacle_broadcaster;

        tf::TransformListener tflistener;
        
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub_encoder, sub_scan, sub_coefficient, sub_cluster, sub_goal_;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub_cmd, pub_odom, pub_ShortestDistance, pub_PV, pub_PP, pub_goal_;

        std_msgs::Header header_;

        geometry_msgs::PoseStamped goal_;

        ros::Time encoder_time_pre, potential_time_pre, manage_time, manage_time_pre;
        geometry_msgs::Twist encoder_value, cmd;
        nav_msgs::Odometry odom, odom_pre, odom_msg;
        double bottom_v = 0,bottom_omega = 0;
        bool odometry_firsttime = true;
        sensor_msgs::LaserScan scan, scan_msg;
        std::vector<std::vector<float>> obstacle;
        geometry_msgs::Vector3 U_pre, ShortestDistance;

        geometry_msgs::PoseWithCovarianceStamped pwcs_msg;

        int path_planning_id = POTENTIAL_METHOD;

        bool encoder_first = false, scan_first = false;

        double error_pos_x_pre = 0.0, error_pos_y_pre = 0.0, error_vel_pre = 0.0, error_angular_pre = 0.0, integral_vel_error = 0.0, integral_angular_error = 0.0;
        double encoder_deltatime;

        bool zero_potential = true;
        double rho_zero, f_time_pre = -999999999999999;

        double cont_pos_time_pre = -999999999999999, pos_x_pre = 0, pos_y_pre = 0, vel_x_pre = 0, vel_y_pre = 0;

        double start_x,start_y,start_theta,target_distance,target_angle, sub_goal_x , sub_goal_y;
        int done_cont_pos_cnt = 0;
        bool done_cont_pos = true;

        double integral_theta_error = 0;

        double distance_to_obstacle;
        int obstacle_index = 0, obstacle_size = 1000;
        potbot::PotentialValue PV;

        std::vector<int> exploration_arr;
        //std::vector<geometry_msgs::Vector3> robot_path;
        nav_msgs::Path robot_path;
        std::vector<int> centered;
        geometry_msgs::Vector3 sub_start;
        int robot_path_index = 0, robot_path_index_pre = 0;
        
        ros::Time path_update_timestamp;

        std::vector<double> scan_range_maximum_angle;
        int scan_range_maximum_angle_size = 10, scan_range_maximum_angle_index = 0;
        bool moving_average = false, wall_exists = false;
        double coe_x, coe_y, coe_0;

        potbot::ClassificationVelocityData pcl_cluster;

        std::string PATH_PLANNING_METHOD, PATH_PLANNING_FILE;
        bool ANGLE_CORRECTION, PID_CONTROL, USE_AMCL, USE_RVIZ;
        double MAX_VELOCITY, MAX_ANGULAR, TARGET_POSITION_X, TARGET_POSITION_Y;
        double GAIN_PROPORTIONAL, GAIN_INTEGRAL, GAIN_DIFFERENTIAL;
        double TARGET_POSITION_MARGIN, PATH_CREATE_PERIOD, POTENTIAL_FIELD_WIDTH, POTENTIAL_FIELD_DIVDE_X, POTENTIAL_FIELD_DIVDE_Y;
        double POTENTIAL_BIAS_COEFFICIENT_0, POTENTIAL_BIAS_COEFFICIENT_1, AHEAD_PATH, EXCLUDE_LRF;
        bool CALCULATE_BIAS;

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
	    void encoder_callback(const geometry_msgs::Twist& msg);
        void encoder_callback_sim(const nav_msgs::Odometry& msg);
        void pwcs_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void scan_callback(const sensor_msgs::LaserScan& msg);
        void coefficient_callback(const std_msgs::Float32& msg);
        void cluster_callback(const potbot::ClassificationVelocityData& msg);
        void goal_callback(const geometry_msgs::PoseStamped& msg);
        void get_topic();

        void manage();

        void transform_obstacle_pos();

        geometry_msgs::Vector3 U_xd(double robot_x, double robot_y);
        geometry_msgs::Vector3 U_o(double robot_x, double robot_y);
        geometry_msgs::Vector3 U(double robot_x, double robot_y);

        geometry_msgs::Vector3 F_xd();
        geometry_msgs::Vector3 rho_x(double robot_x, double robot_y);
        geometry_msgs::Vector3 F_o();
        geometry_msgs::Vector3 F();
        void potential();

        void create_exploration_idx(int width);
        void path_planning();
        void line_following();

        void cont_vel(double vel_x, double vel_y);
        void cont_pos(double goal_x, double goal_y);

        void publishShortestDistance();
        void publishPotentialValue();
        void publishPathPlan();
};