//include haeders
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <autonomous_mobile_robot_2022/PotentialValue.h>
#include <autonomous_mobile_robot_2022/PathPlan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <autonomous_mobile_robot/ClassificationVelocityData.h>

//クラスの定義
class PotentialMethodClass{

    private:
        tf::TransformBroadcaster robotState_broadcaster;
        tf::TransformBroadcaster LRF_broadcaster;
        tf::TransformBroadcaster StereoCamera_broadcaster;
        tf::TransformBroadcaster obstacle_broadcaster;

        tf::TransformListener tflistener;
        
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub_encoder, sub_scan, sub_coefficient, sub_cluster;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub_cmd, pub_odom, pub_ShortestDistance, pub_PV, pub_PP;

        ros::Time encoder_time_pre, potential_time_pre, manage_time, manage_time_pre;
        ros::WallTime start_time;
        geometry_msgs::Twist encoder_value, cmd;
        nav_msgs::Odometry odom, odom_pre, odom_msg;
        double bottom_v = 0,bottom_omega = 0;
        bool odometry_firsttime = true;
        sensor_msgs::LaserScan scan, scan_msg;
        std::vector<std::vector<float>> obstacle;
        geometry_msgs::Vector3 U_pre, ShortestDistance;

        geometry_msgs::PoseWithCovarianceStamped pwcs_msg;

        int robot_id = 0, path_planning_id = 1;

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
        autonomous_mobile_robot_2022::PotentialValue PV;

        std::vector<int> exploration_arr;
        std::vector<geometry_msgs::Vector3> robot_path;
        std::vector<int> centered;
        bool path_update = true;
        geometry_msgs::Vector3 sub_start;
        int robot_path_index = 0, robot_path_index_pre = 0;
        autonomous_mobile_robot_2022::PathPlan PP;
        
        ros::Time path_update_timestamp;

        std::vector<double> scan_range_maximum_angle;
        int scan_range_maximum_angle_size = 10, scan_range_maximum_angle_index = 0;
        bool moving_average = false, wall_exists = false;
        double coe_x, coe_y, coe_0;

        autonomous_mobile_robot::ClassificationVelocityData pcl_cluster;

        std::string ROBOT_NAME, PATH_PLANNING_METHOD, PATH_PLANNING_FILE;
        bool IS_SIMULATOR, PUBLISH_COMMAND, ANGLE_CORRECTION, PID_CONTROL, USE_AMCL;
        double MAX_VELOCITY, MAX_ANGULAR, TARGET_POSITION_X, TARGET_POSITION_Y;
        double GAIN_PROPORTIONAL, GAIN_INTEGRAL, GAIN_DIFFERENTIAL;
        double PATH_TRACKING_MARGIN, PATH_CREATE_PERIOD, POTENTIAL_FIELD_WIDTH, POTENTIAL_FIELD_DIVDE_X, POTENTIAL_FIELD_DIVDE_Y;
        double POTENTIAL_BIAS_COEFFICIENT_0, POTENTIAL_BIAS_COEFFICIENT_1;

    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        PotentialMethodClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~PotentialMethodClass();
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
        void cluster_callback(const autonomous_mobile_robot::ClassificationVelocityData& msg);
        void get_topic();

        void manage();

        void odometry();

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

        //センサデータ送信
        void publishcmd();//データ送信
        void publishodom();
        void publishShortestDistance();
        void publishPotentialValue();
        void publishPathPlan();
        //データクリア
        //void clearMessages();
};
