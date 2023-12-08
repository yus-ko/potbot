//include haeders
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <potbot_msgs/PotentialValue.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

//クラスの定義
class rvizClass{

    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub_PV, sub_odom;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub_marker, pub_marker_pp, pub_path;
        
        potbot_msgs::PotentialValue PV;
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::MarkerArray marker_array_pp;

        nav_msgs::Odometry odom;
        ros::Time CreatePath_time_pre;
        nav_msgs::Path robot_traj_;

        double POTENTIAL_VALUE_PLOT_LIMIT, POTENTIAL_VALUE_PLOT_SHRINK_SCALE;

    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        rvizClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~rvizClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
	    void PotentialValue_callback(const potbot_msgs::PotentialValue& msg);
        void odom_callback(const nav_msgs::Odometry& msg);

        void manage();

        void addMarker();
        void CreateTraj();
        
        //センサデータ送信
        void publishMarker();//データ送信
        void publishTraj();
        
        //データクリア
        //void clearMessages();
};
