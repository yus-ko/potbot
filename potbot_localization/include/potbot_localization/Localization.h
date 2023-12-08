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

//クラスの定義
class LocalizationClass{

    private:
        
        //センサーデータ
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_odom_, sub_map_, sub_inipose_, sub_goal_, sub_point_, sub_scan_;
        //送信データ
        ros::NodeHandle nhPub_;
		ros::Publisher pub_particle_, pub_localmap_, pub_odom_;

        tf2_ros::TransformBroadcaster broadcaster_;

        sensor_msgs::LaserScan scan_;

        nav_msgs::OccupancyGrid world_map_, local_map_;

        geometry_msgs::PointStamped point_;
        geometry_msgs::PoseStamped goal_;
        geometry_msgs::PoseWithCovarianceStamped initial_pose_;
        
        bool encoder_first_ = false;
        std_msgs::Header header_, header_pre_, resampling_time_;

        int localization_method_id_ = 0;

        potbot_msgs::beego_encoder beggo_;

        geometry_msgs::Twist encoder_value_;
        nav_msgs::Odometry odom_;
        
        
        visualization_msgs::MarkerArray particles_;
        std::vector<double> weights_;
        int particle_num_ = 100;
        double robot_pose_x_ = 0, robot_pose_y_ = 0, robot_pose_theta_ = 0;

        std::random_device seed_gen_;

        int maximum_likefood_particle_id_ = 0;

        int Tn_=30;
        double square_width_=0.1;

        dynamic_reconfigure::Server<potbot_localization::LocalizationConfig> server_;
  	    dynamic_reconfigure::Server<potbot_localization::LocalizationConfig>::CallbackType f_;

        std::string ROBOT_NAME, LOCALIZATION_METHOD, TOPIC_SCAN, TOPIC_ODOM;
        // std::string FRAME_ID_GLOBAL, FRAME_ID_ROBOT_BASE, FRAME_ID_LIDAR;
        bool IS_SIMULATOR, USE_RVIZ;
        double COVARIANCE_VV, COVARIANCE_VOMEGA, COVARIANCE_OMEGAOMEGA, INITIAL_POSE_X, INITIAL_POSE_Y, INITIAL_POSE_THETA;

        void __param_callback(const potbot_localization::LocalizationConfig& param, uint32_t level);
        void __scan_callback(const sensor_msgs::LaserScan& msg);
        void __odom_callback(const nav_msgs::Odometry& msg);

    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        LocalizationClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~LocalizationClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        //void beego_encoder_callback(const potbot_msgs::beego_encoder& msg);
	    //void encoder_callback(const geometry_msgs::Twist& msg);
        
        void map_callback(const nav_msgs::OccupancyGrid& msg);
        void inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void goal_callback(const geometry_msgs::PoseStamped& msg);
        void point_callback(const geometry_msgs::PointStamped& msg);

        void set_pose(geometry_msgs::PoseWithCovarianceStamped pose);
        
        double match_rate(nav_msgs::OccupancyGrid local,nav_msgs::OccupancyGrid world);
        
        void manage();
        void odometry();
        double draw_gaussian(double mu, double sigma);
        void reset_particle();
        void create_particle();
        void resampling();
        void puclish_odom();
        void tf_broadcast();

        
};
