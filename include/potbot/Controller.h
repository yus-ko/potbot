//include haeders
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//クラスの定義
class ControllerClass{

    private:
        
        //センサーデータ
		ros::NodeHandle nhSub_;
		ros::Subscriber sub_odom_;
        //送信データ
        ros::NodeHandle nhPub_;
		ros::Publisher pub_cmd_;

        int robot_id_ = 0;
        
        geometry_msgs::Twist cmd_;
        double robot_pose_x_ = 0, robot_pose_y_ = 0, robot_pose_theta_ = 0;

        std::vector<geometry_msgs::Vector3> robot_path_;
        int robot_path_index_ = 0;

        std::string ROBOT_NAME;
        bool IS_SIMULATOR;
        std::string PATH_PLANNING_FILE;

    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        ControllerClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~ControllerClass();
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //in methods.cpp
        //--センサーデータ受信
        void odom_callback(const nav_msgs::Odometry& msg);
        
        void manage();
        void controller();
        void calculate_cmd();
        void publishcmd();

        
};
