//include haeders
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//クラスの定義
class TFClass{

    private:
        tf::TransformBroadcaster robotState_broadcaster;
        tf::TransformBroadcaster LRF_broadcaster;
        tf::TransformBroadcaster StereoCamera_broadcaster;
        tf::TransformBroadcaster obstacle_broadcaster;

		ros::NodeHandle nhSub;
		ros::Subscriber sub_encoder, sub_scan;
        

        ros::Time manage_time, manage_time_pre;
        nav_msgs::Odometry odom, odom_pre, odom_msg;
        sensor_msgs::LaserScan scan, scan_msg;
        geometry_msgs::PoseWithCovarianceStamped pwcs_msg;

    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        TFClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~TFClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        void pwcs_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void scan_callback(const sensor_msgs::LaserScan& msg);
        void get_topic();

        void manage();

};
