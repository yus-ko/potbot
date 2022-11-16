//多重インクルード防止
#ifndef INCLUDE_CONV_CAM_DATA_CLASS
#define INCLUDE_CONV_CAM_DATA_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//for image processing on ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//self msg
#include <autonomous_mobile_robot_2022/SensorMapData.h>
#include <autonomous_mobile_robot_2022/MaskImageData.h>
// msg
#include <geometry_msgs/QuaternionStamped.h>
//床面推定
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
// #include <autonomous_mobile_robot/convCamDataConfig.h>
//クラスの定義
class convCamDataClass{

    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
        cv_bridge::CvImagePtr bridgeImage;
        //送信データ
		ros::NodeHandle nhPub1,nhPub2;
        ros::Publisher pubConv,pubMask;
        // カメラパラメータ
        float f;//焦点距離、画像パラメータ
        float camHeight;//床面からの高さ
        //マップパラメータ
        float mapW;//width[m]
        float mapH;//height[m]
        float mapR;//resolution[m]
        int mapWi;//マップサイズWidth[pixel]
        int mapHi;//マップサイズHeight[pixel]

    protected:
        autonomous_mobile_robot_2022::SensorMapData smd;
        autonomous_mobile_robot_2022::MaskImageData mid;
        
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        convCamDataClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~convCamDataClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        bool subscribeSensorData();//データ受信
        void sensor_callback(const sensor_msgs::ImageConstPtr& msg);
        //--manage
        void manage();
        void create2dMap();//2次元ローカルマップ作成, マスク画像作成
        //--座標変換
        //センサ座標系ー＞マップ座標系
        bool convertToGrid(const float& x,const float& y,int& xg,int& yg);
        //センサデータ送信
        void publishConvCamData();//データ送信
        //マスク画像送信
        void publishMaskImage();//データ送信
        //データクリア
        void clearMessages();
};
#endif