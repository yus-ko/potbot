//include haeders
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

//点群
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <autonomous_mobile_robot_2022/PointCloudData.h>
#include <autonomous_mobile_robot_2022/ClusterData.h>
//#include <beego_control/beego_encoder.h>

//クラスの定義
class PointCloudClass{

    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
		ros::Subscriber sub_encoder, sub_depthimage, sub_odom;
        //sensor_msgs::PointCloud2 rawPC2,PC2;
        //点群データ
        pcl::PointCloud<pcl::PointXYZ> rawcloud;
        autonomous_mobile_robot_2022::PointCloudData cloud,cloudfromimage;
        //クラスタリング
        autonomous_mobile_robot_2022::ClusterData cluster, cluster_nondup;
        //送信データ
		ros::NodeHandle nhPub1;
        ros::Publisher pubpc, pubpc1, pub_clus;
	
        double DISTANCE_TO_OBJECT, ANGLE_TO_OBJECT, OBJECTSIZE_HEIGHT, OBJECTSIZE_WIDTH, OBJECTSIZE_DEPTH, CAMERAPOS_HEIGHT, PIXEL_TO_XYZ;
        bool IS_MOVING;

        double distance_traveled_robot = 0.0;   //移動距離
        bool encoder_firsttime = false;

	    ros::Time encoder_time_pre;

        cv_bridge::CvImagePtr bridgeImage;

        nav_msgs::Odometry odom;
        
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        PointCloudClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~PointCloudClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void odom_callback(const nav_msgs::Odometry& msg);
	    //void encoder_callback(const beego_control::beego_encoder& msg);
        void depthimage_callback(const sensor_msgs::Image& msg);
        //--manage
        void manage();
        //点群の抜出
        void Extract();
        //クラスタリング
        void Clustering();
        //センサデータ送信
        void publishPointCloud();//データ送信
        //データクリア
        void clearMessages();
};
